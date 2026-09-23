/* SPDX-License-Identifier: MIT */
/* Encode contexts: raw NV12 in, whole-frame submit, coded bytes out. */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <stddef.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <va/va.h>
#include <va/va_backend.h>
#include <va/va_enc_h264.h>
#include <va/va_enc_hevc.h>

#include "va_mtkvcp.h"

#define ENC_OUT_TYPE V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
#define ENC_CAP_TYPE V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
#define V4L2_CID_MPEG_MTK_PADDED_NV12_CHROMA (V4L2_CTRL_CLASS_CODEC | 0x20f0)

int mtkvcp_padded_uv_enabled(void)
{
    const char *v = getenv("MTK_VCP_VA_PADDED_NV12_UV");

    return v && strcmp(v, "1") == 0;
}

/* CLOCK_MONOTONIC milliseconds, for the frame-rate accounting below. */
static uint64_t mtkvcp_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)ts.tv_nsec / 1000000ull;
}
#define ENC_TIMEOUT_MS 15000

static int mtkvcp_ioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do {
        r = ioctl(fd, req, arg);
    } while (r < 0 && errno == EINTR);
    return r < 0 ? -errno : 0;
}

static int mtkvcp_s_ctrl(int fd, uint32_t id, int32_t val)
{
    struct v4l2_control c;
    memset(&c, 0, sizeof(c));
    c.id = id;
    c.value = val;
    return mtkvcp_ioctl(fd, VIDIOC_S_CTRL, &c);
}

static VAStatus mtkvcp_claim_hw(struct mtkvcp_drv *d, int ci)
{
    if (d->hw_owner_ctx == ci)
        return VA_STATUS_SUCCESS;
    if (d->hw_owner_ctx >= 0)
        return VA_STATUS_ERROR_HW_BUSY;
    d->hw_owner_ctx = ci;
    return VA_STATUS_SUCCESS;
}

static void mtkvcp_release_hw(struct mtkvcp_drv *d, int ci)
{
    if (d->hw_owner_ctx == ci)
        d->hw_owner_ctx = -1;
}

VAStatus mtkvcp_enc_create(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    const char *node = getenv("MTK_VCP_VA_ENC_NODE");
    struct v4l2_format g;
    int r, i;
    int fw;
    if (!node || !*node)
        node = MTKVCP_ENC_NODE;
    c->vfd = mtkvcp_v4l2_open(node);
    if (c->vfd < 0)
        return VA_STATUS_ERROR_OPERATION_FAILED;
    if (c->rgb_in)
        c->out_fourcc = mtkvcp_rgb_fourcc(c->rgb_in);
    {
        /* Match the export side: pitch the raw-input ring at a 64-byte
         * aligned width so buffers exported for client writes can be
         * imported by the GPU and handed straight back. */
        unsigned int px_align =
            c->rgb_in ? 16u :
            (c->out_fourcc == V4L2_PIX_FMT_P010 ? 32u : 64u);

        fw = (int)(((unsigned)c->width + px_align - 1u) &
                   ~(px_align - 1u));
        r = mtkvcp_v4l2_s_fmt(c->vfd, ENC_OUT_TYPE, c->out_fourcc,
                              fw, c->height, NULL);
    }
    /* The ring is pitched at the aligned width so exported buffers can be
     * imported by the GPU (64-byte plane pitch on Panfrost), but the coded
     * picture must stay the client's visible size. The crop selection is
     * the V4L2 contract for that: coded geometry from S_FMT, visible
     * geometry from S_SELECTION, and the firmware lays chroma out at
     * buf stride * visible height, which is exactly the offset exported
     * surfaces report. */
    if (fw != c->width) {
        struct v4l2_selection sel;

        memset(&sel, 0, sizeof(sel));
        sel.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        sel.target = V4L2_SEL_TGT_CROP;
        sel.r.left = 0;
        sel.r.top = 0;
        sel.r.width = (uint32_t)c->width;
        sel.r.height = (uint32_t)c->height;
        if (mtkvcp_ioctl(c->vfd, VIDIOC_S_SELECTION, &sel) < 0)
            mtkvcp_log("enc crop %dx%d refused errno=%d (coded size "
                       "stays %dx%d)", c->width, c->height, errno, fw,
                       c->height);
        else
            mtkvcp_log("enc crop set %dx%d inside %dx%d", c->width,
                       c->height, fw, c->height);
    }
    c->enc_visible_width = c->width;
    c->enc_visible_height = c->height;
    if (r < 0) {
        mtkvcp_log("enc OUT S_FMT failed errno=%d", r);
        close(c->vfd);
        c->vfd = -1;
        return r == -EBUSY ? VA_STATUS_ERROR_HW_BUSY :
                             VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED;
    }
    memset(&g, 0, sizeof(g));
    g.type = ENC_OUT_TYPE;
    if (mtkvcp_ioctl(c->vfd, VIDIOC_G_FMT, &g) == 0)
        c->out_stride = (int)g.fmt.pix_mp.plane_fmt[0].bytesperline;
    if (c->out_stride < c->width)
        c->out_stride = c->width;
    mtkvcp_log("enc_create %dx%d out=%c%c%c%c stride=%d rgb_in=%d",
               c->width, c->height, c->out_fourcc & 0xff,
               (c->out_fourcc >> 8) & 0xff, (c->out_fourcc >> 16) & 0xff,
               (c->out_fourcc >> 24) & 0xff, c->out_stride, c->rgb_in);
    /*
     * The packed-RGB capture path can hand the imported dma-buf straight
     * to the encoder, which removes the userspace copy between the
     * post-processing step and the driver. A vb2 queue has a single
     * memory type, so the whole OUTPUT ring moves to DMABUF when that
     * path is active. If no dma-heap is present the ring stays in MMAP
     * memory and frames are copied as before.
     */
    if (c->rgb_in && mtkvcp_zerocopy_enabled()) {
        size_t need = (size_t)c->out_stride * (size_t)c->height;
        int probe_fd = mtkvcp_dma_heap_alloc(need);

        c->out_dmabuf = probe_fd >= 0;
        if (probe_fd >= 0)
            close(probe_fd);
        if (!c->out_dmabuf)
            mtkvcp_log("enc_create: no dma-heap, keeping MMAP OUTPUT "
                       "(frames will be copied)");
    } else if (c->rgb_in) {
        mtkvcp_log("enc_create: zero-copy disabled by "
                   "MTK_VCP_VA_ZEROCOPY=0, keeping MMAP OUTPUT");
    }
    /* NV12 contexts get the same treatment: clients that export their
     * input surfaces for writing (FFmpeg's vaapi encoder) hand the
     * buffer back through the VPP-less alias path, and the firmware can
     * DMA from it directly. Unlike the packed-RGB path, whose alias is a
     * compositor buffer that the compositor may repaint at any moment,
     * these buffers are written only through the VA surface itself and
     * the client reuses a surface only after syncing the coded output,
     * so the alias is safe without an opt-in. Without a dma-heap the
     * ring stays MMAP and frames are copied as before. */
    if (!c->rgb_in) {
        size_t need = (size_t)c->out_stride * (size_t)c->height * 3u / 2u;
        int probe_fd = mtkvcp_dma_heap_alloc(need);

        c->out_dmabuf = probe_fd >= 0;
        if (probe_fd >= 0)
            close(probe_fd);
        if (!c->out_dmabuf)
            mtkvcp_log("enc_create: no dma-heap, keeping MMAP OUTPUT "
                       "(frames will be copied)");
    }
    r = mtkvcp_v4l2_reqbufs_mem(c->vfd, ENC_OUT_TYPE, MTKVCP_OUT_BUFS_ENC,
                                c->out_dmabuf ? V4L2_MEMORY_DMABUF :
                                                V4L2_MEMORY_MMAP);
    if (r < MTKVCP_OUT_BUFS_ENC) {
        close(c->vfd);
        c->vfd = -1;
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    c->out_count = MTKVCP_OUT_BUFS_ENC;
    d->enc_dmabuf_ring = c->out_dmabuf;
    if (c->out_dmabuf) {
        /* DMABUF memory: no MMAP of the ring, no userspace copy. The
         * buffers are created on demand from the surface handed to
         * EndPicture. */
        memset(c->out_len, 0, sizeof(c->out_len));
        for (i = 0; i < c->out_count; i++) {
            c->out_map[i] = NULL;
            c->out_free[i] = 1;
            c->out_stage_fd[i] = -1;
            c->out_stage_map[i] = NULL;
            c->out_stage_len[i] = 0;
            c->out_slot_key[i] = 0;
        }
        goto out_ring_done;
    }
    for (i = 0; i < c->out_count; i++) {
        struct v4l2_buffer b;
        struct v4l2_plane pl;
        memset(&b, 0, sizeof(b));
        memset(&pl, 0, sizeof(pl));
        b.type = ENC_OUT_TYPE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = (uint32_t)i;
        b.length = 1;
        b.m.planes = &pl;
        if (mtkvcp_ioctl(c->vfd, VIDIOC_QUERYBUF, &b) < 0) {
            int j;
            for (j = 0; j < i; j++)
                if (c->out_map[j]) {
                    munmap(c->out_map[j], c->out_len[j]);
                    c->out_map[j] = NULL;
                }
            close(c->vfd);
            c->vfd = -1;
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        c->out_len[i] = pl.length;
        c->out_map[i] = mmap(NULL, pl.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, c->vfd, pl.m.mem_offset);
        if (c->out_map[i] == MAP_FAILED) {
            int j;
            c->out_map[i] = NULL;
            for (j = 0; j < i; j++)
                if (c->out_map[j]) {
                    munmap(c->out_map[j], c->out_len[j]);
                    c->out_map[j] = NULL;
                }
            close(c->vfd);
            c->vfd = -1;
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        c->out_free[i] = 1;
    }
out_ring_done:
    r = mtkvcp_v4l2_s_fmt(c->vfd, ENC_CAP_TYPE, c->cap_fourcc,
                          c->width, c->height, NULL);
    if (r < 0)
        mtkvcp_log("enc CAP S_FMT failed errno=%d", r);
    if (r < 0) {
        int j, st;
        for (j = 0; j < c->out_count; j++)
            if (c->out_map[j]) {
                munmap(c->out_map[j], c->out_len[j]);
                c->out_map[j] = NULL;
            }
        close(c->vfd);
        c->vfd = -1;
        st = r == -EBUSY ? VA_STATUS_ERROR_HW_BUSY :
                           VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED;
        return st;
    }
    r = mtkvcp_v4l2_reqbufs(c->vfd, ENC_CAP_TYPE, MTKVCP_CAP_BUFS_ENC);
    if (r < MTKVCP_CAP_BUFS_ENC) {
        int j;
        for (j = 0; j < c->out_count; j++)
            if (c->out_map[j]) {
                munmap(c->out_map[j], c->out_len[j]);
                c->out_map[j] = NULL;
            }
        close(c->vfd);
        c->vfd = -1;
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    c->cap_count = MTKVCP_CAP_BUFS_ENC;
    for (i = 0; i < c->cap_count; i++) {
        struct v4l2_buffer b;
        struct v4l2_plane pl;
        memset(&b, 0, sizeof(b));
        memset(&pl, 0, sizeof(pl));
        b.type = ENC_CAP_TYPE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = (uint32_t)i;
        b.length = 1;
        b.m.planes = &pl;
        if (mtkvcp_ioctl(c->vfd, VIDIOC_QUERYBUF, &b) < 0)
            break;
        c->enccap_len[i] = pl.length;
        c->enccap_map[i] = mmap(NULL, pl.length, PROT_READ | PROT_WRITE,
                                MAP_SHARED, c->vfd, pl.m.mem_offset);
        if (c->enccap_map[i] == MAP_FAILED) {
            c->enccap_map[i] = NULL;
            break;
        }
        c->enccap_free[i] = 1;
        /* Pre-queue every coded buffer. */
        memset(&b, 0, sizeof(b));
        memset(&pl, 0, sizeof(pl));
        b.type = ENC_CAP_TYPE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = (uint32_t)i;
        b.length = 1;
        b.m.planes = &pl;
        if (mtkvcp_ioctl(c->vfd, VIDIOC_QBUF, &b) < 0)
            break;
    }
    if (i < c->cap_count) {
        int j;
        for (j = 0; j < c->out_count; j++)
            if (c->out_map[j]) {
                munmap(c->out_map[j], c->out_len[j]);
                c->out_map[j] = NULL;
            }
        for (j = 0; j < i; j++)
            if (c->enccap_map[j]) {
                munmap(c->enccap_map[j], c->enccap_len[j]);
                c->enccap_map[j] = NULL;
            }
        close(c->vfd);
        c->vfd = -1;
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    mtkvcp_v4l2_subscribe(c->vfd, V4L2_EVENT_EOS);
    /* The firmware picks the emitted H.264 profile from the V4L2 codec
     * control, whose menu is Baseline/Main/High and whose default is
     * High. Nothing set it before, so every stream came out as High
     * regardless of the VA profile the client asked for. Map the VA
     * profile onto the V4L2 ordinal here, before STREAMON (the control
     * is static: the driver refuses codec controls once streaming).
     * ConstrainedBaseline maps to the Baseline ordinal: the firmware
     * writes profile_idc 66, which is what the RDP AVC420 path wants
     * (krdp asks for AV_PROFILE_H264_CONSTRAINED_BASELINE). */
    if (c->cap_fourcc == V4L2_PIX_FMT_H264) {
        int va_profile = d->configs[c->config].profile;
        int v4l2_profile = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH;
        switch (va_profile) {
        case VAProfileH264ConstrainedBaseline:
            v4l2_profile = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
            break;
        case VAProfileH264Main:
            v4l2_profile = V4L2_MPEG_VIDEO_H264_PROFILE_MAIN;
            break;
        default:
            break;
        }
        r = mtkvcp_s_ctrl(c->vfd, V4L2_CID_MPEG_VIDEO_H264_PROFILE,
                          v4l2_profile);
        if (r < 0)
            mtkvcp_log("enc h264_profile=%d refused errno=%d (firmware "
                       "default stays in effect)", v4l2_profile, -r);
        else
            mtkvcp_log("enc h264_profile=%d (va_profile=%d)",
                       v4l2_profile, va_profile);
    }
    c->enc_bitrate = 2000000;
    c->enc_framerate_num = 30;
    c->enc_framerate_den = 1;
    c->enc_gop = 30;
    c->enc_applied_bitrate = 0;
    c->enc_applied_gop = 0;
    c->enc_applied_fps_num = 0;
    c->enc_applied_fps_den = 0;
    c->enc_params_dirty = 1;
    for (i = 0; i < MTKVCP_ENC_MAX_JOBS; i++)
        c->enc_jobs[i].in_use = 0;
    if (!c->out_dmabuf) {
        for (i = 0; i < MTKVCP_OUT_BUFS_ENC; i++) {
            c->out_stage_fd[i] = -1;
            c->out_stage_map[i] = NULL;
            c->out_stage_len[i] = 0;
            c->out_slot_key[i] = 0;
        }
    }
    c->enc_outstanding = 0;
    return VA_STATUS_SUCCESS;
}

VAStatus mtkvcp_enc_begin(struct mtkvcp_drv *d, int ci, int si)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    struct mtkvcp_surface *s = &d->surfaces[si];
    size_t need;
    int padded_surface;
    if (c->error)
        return VA_STATUS_ERROR_ENCODING_ERROR;
    if (c->enc_target >= 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (s->ctx < 0) {
        /* Clients size the context to the aligned frame and the
         * surfaces to the visible frame, so both may differ from the
         * context by up to the alignment step: krdp hands over
         * 2460x1080 surfaces for a 2464x1088 context. Reject only a
         * real mismatch; the padding is zeroed when the frame is
         * copied into the V4L2 OUTPUT buffer. */
        if (s->width > c->width || s->width + 16 < c->width ||
            s->height > c->height || s->height + 16 < c->height) {
            mtkvcp_log("enc_begin size mismatch: surface %dx%d ctx %dx%d "
                       "(kind=%d)", s->width, s->height, c->width, c->height,
                       s->kind);
            return VA_STATUS_ERROR_INVALID_SURFACE;
        }
        s->ctx = ci;
        s->kind = MTKVCP_SURF_ENC_INPUT;
    } else if (s->ctx != ci || s->kind != MTKVCP_SURF_ENC_INPUT) {
        mtkvcp_log("enc_begin bind mismatch: surface %dx%d ctx %dx%d "
                   "s->ctx=%d ci=%d kind=%d", s->width, s->height,
                   c->width, c->height, s->ctx, ci, s->kind);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    padded_surface = mtkvcp_padded_uv_enabled() && !c->rgb_in &&
        c->out_dmabuf && c->height % 32 == 0 &&
        s->prime_fd >= 0 && s->height < c->height &&
        s->export_bh == c->height &&
        s->export_stride == c->out_stride &&
        s->export_uvoff == s->export_stride * c->height;
    if (padded_surface && !c->enc_padded_uv) {
        int ret;

        if (c->streaming || c->enc_outstanding)
            return VA_STATUS_ERROR_INVALID_SURFACE;
        ret = mtkvcp_s_ctrl(c->vfd,
                            V4L2_CID_MPEG_MTK_PADDED_NV12_CHROMA, 1);
        if (ret < 0) {
            mtkvcp_log("enc padded NV12 UV control refused errno=%d", -ret);
            return VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED;
        }
        c->enc_padded_uv = 1;
        mtkvcp_log("enc padded NV12 UV enabled for %dx%d surface",
                   s->width, s->height);
    }
    /* A context cannot mix cropped sources with different UV origins. */
    if (c->enc_padded_uv && !padded_surface)
        return VA_STATUS_ERROR_INVALID_SURFACE;
    if (!c->enc_padded_uv && s->prime_fd >= 0 &&
        s->export_uvoff > s->export_stride * s->height)
        return VA_STATUS_ERROR_INVALID_SURFACE;
    /* FFmpeg may align the encode context (2464x1088) while its input
     * surfaces keep the display size (2460x1080). Exported NV12 chroma
     * starts after the surface's 1080 luma rows; VENC must use the same
     * visible height to read the correct plane. */
    if (s->width != c->enc_visible_width ||
        s->height != c->enc_visible_height) {
        struct v4l2_selection sel;
        int crop_ret;

        if (c->streaming || c->enc_outstanding) {
            mtkvcp_log("enc crop change after STREAMON: %dx%d -> %dx%d",
                       c->enc_visible_width, c->enc_visible_height,
                       s->width, s->height);
            return VA_STATUS_ERROR_INVALID_SURFACE;
        }
        memset(&sel, 0, sizeof(sel));
        sel.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        sel.target = V4L2_SEL_TGT_CROP;
        sel.r.width = s->width;
        sel.r.height = s->height;
        crop_ret = mtkvcp_ioctl(c->vfd, VIDIOC_S_SELECTION, &sel);
        if (crop_ret < 0 || sel.r.width != (uint32_t)s->width ||
            sel.r.height != (uint32_t)s->height) {
            mtkvcp_log("enc crop %dx%d failed: ret=%d actual=%dx%d",
                       s->width, s->height, crop_ret,
                       sel.r.width, sel.r.height);
            return VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED;
        }
        c->enc_visible_width = s->width;
        c->enc_visible_height = s->height;
        mtkvcp_log("enc crop %dx%d for %dx%d context",
                   s->width, s->height, c->width, c->height);
    }
    if (!s->enc_data && s->prime_fd < 0 &&
        !(c->out_dmabuf && s->alias_fd >= 0)) {
        /* Pre-bound surfaces arrive without storage; back them now. */
        need = c->rgb_in ?
               (size_t)mtkvcp_rgb_stride(c->width) * (size_t)c->height :
               (size_t)c->width * (size_t)c->height * 3u / 2u;
        s->enc_data = malloc(need ? need : 1);
        if (!s->enc_data)
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        memset(s->enc_data, 0, need ? need : 1);
        s->enc_size = need;
        s->enc_stride = c->rgb_in ? mtkvcp_rgb_stride(c->width) : c->width;
        s->enc_rgb = c->rgb_in ? 1 : 0;
    }
    /* A surface the client exported in order to render into it carries
     * its content in the exported dma-buf. When the OUTPUT ring runs in
     * DMABUF memory the buffer can go straight to the firmware, exactly
     * like the capture path's alias; otherwise the submit step copies
     * from the mapping. */
    if (!c->rgb_in && c->out_dmabuf && s->prime_fd >= 0 &&
        s->alias_fd < 0 && s->export_stride > 0) {
        s->alias_fd = fcntl(s->prime_fd, F_DUPFD_CLOEXEC, 0);
        if (s->alias_fd >= 0) {
            s->alias_stride = s->export_stride;
            s->alias_size = (int)s->imp_size;
            s->alias_key = 0;
            s->enc_rgb = 0;
        }
    }
    /* A surface the VPP filled in packed-RGB mode arrives with the RGB
     * staging already in place; anything else must match the mode the
     * V4L2 OUTPUT format was negotiated for.
     */
    if (c->rgb_in && s->enc_rgb != 1) {
        mtkvcp_log("enc_begin: packed-RGB input expected but surface %d "
                   "holds %s", si, s->enc_rgb == 0 ? "NV12" : "nothing");
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    if (!c->rgb_in && s->enc_rgb == 1) {
        mtkvcp_log("enc_begin: surface %d holds packed RGB but the context "
                   "encodes NV12", si);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    c->enc_target = si;
    c->enc_coded_buf = -1;
    c->enc_force_idr = 0;
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_enc_apply_params(struct mtkvcp_context *c)
{
    struct v4l2_streamparm parm;
    unsigned int fps_num, fps_den;
    int r, failed = 0;
    int bitrate_forced = 0;
    /* Quality-only clients (KPipeWire passes global_quality and no
     * bitrate) need a bitrate anyway: the firmware implements CBR only.
     * Convert the quality factor into bits per pixel per frame and scale
     * by the real frame size and rate, so the result tracks resolution
     * the way a constant-quality encoder would.
     *
     * The curve is anchored so that factor 26 (a middling H.264 QP, and
     * the value ffmpeg falls back to for non-CQP modes) yields 0.05 bpp:
     * 1080p30 lands on 3.1 Mbit/s, which is the low-latency target krdp
     * itself derives for that resolution. The factor is clamped to
     * 0.01..MTK_VCP_VA_MAX_BPP (default 0.30) bpp.
     *
     * Two environment overrides exist because the derived number is only
     * a guess at what the link should carry:
     *   MTK_VCP_VA_BITRATE=<bps>  force this exact target, ignoring the
     *                             quality factor entirely.
     *   MTK_VCP_VA_MAX_BPP=<x>    raise/lower the clamp ceiling (0<x<=1).
     * The driver accepts 64..100000000 bps, so the ceiling here is the
     * only real limit for a quality-only client such as KPipeWire.
     */
    {
        static int forced_read;
        static unsigned int forced;

        if (!c->enc_bitrate_explicit && !forced_read) {
            const char *v = getenv("MTK_VCP_VA_BITRATE");

            forced_read = 1;
            if (v && *v) {
                long b = strtol(v, NULL, 10);

                if (b >= 64 && b <= 100000000)
                    forced = (unsigned int)b;
                else
                    mtkvcp_log("MTK_VCP_VA_BITRATE=%s out of range, ignored",
                               v);
            }
        }
        if (!c->enc_bitrate_explicit && forced) {
            c->enc_bitrate = forced;
            bitrate_forced = 1;
        }
    }
    if (!c->enc_bitrate_explicit && c->enc_quality_factor && !bitrate_forced) {
        unsigned int qp = c->enc_quality_factor;
        static double max_bpp = -1.0;
        double bpp;
        double fps;
        double bits;

        if (max_bpp < 0.0) {
            const char *v = getenv("MTK_VCP_VA_MAX_BPP");

            max_bpp = 0.30;
            if (v && *v) {
                char *end;
                double d = strtod(v, &end);

                if (end != v && d > 0.0 && d <= 1.0)
                    max_bpp = d;
                else
                    mtkvcp_log("MTK_VCP_VA_MAX_BPP=%s ignored, using %.2f",
                               v, max_bpp);
            }
        }
        if (qp < 1)
            qp = 1;
        if (qp > 51)
            qp = 51;
        bpp = 0.05 * pow(2.0, (26.0 - (double)qp) / 6.0);
        if (bpp < 0.01)
            bpp = 0.01;
        if (bpp > max_bpp)
            bpp = max_bpp;
        fps = c->enc_framerate_den ?
            (double)c->enc_framerate_num / (double)c->enc_framerate_den :
            (double)c->enc_framerate_num;
        if (fps <= 0.0)
            fps = 30.0;
        bits = bpp * (double)c->width * (double)c->height * fps;
        if (bits > 100000000.0)
            bits = 100000000.0;
        if (bits < 64.0)
            bits = 64.0;
        c->enc_bitrate = (unsigned int)bits;
        mtkvcp_log("enc quality factor %u -> %u bps (%.4f bpp, %dx%d @ "
                   "%.3f fps)", c->enc_quality_factor, c->enc_bitrate, bpp,
                   c->width, c->height, fps);
    }
    if (bitrate_forced)
        mtkvcp_log("enc bitrate forced to %u bps by MTK_VCP_VA_BITRATE",
                   c->enc_bitrate);
    fps_num = c->enc_framerate_num ? c->enc_framerate_num : 30;
    fps_den = c->enc_framerate_den ? c->enc_framerate_den : 1;

    /* CBR only: the firmware treats VBR as CBR, so never offer it. Apply
     * only what changed since the last frame and check every ioctl (F9):
     * a refused control must be visible, not silently dropped. */
    if (!c->enc_applied_bitrate) {
        r = mtkvcp_s_ctrl(c->vfd, V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
                          V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
        if (r < 0) {
            mtkvcp_log("enc BITRATE_MODE refused errno=%d", -r);
            failed = 1;
        }
    }
    if (c->enc_bitrate != c->enc_applied_bitrate) {
        r = mtkvcp_s_ctrl(c->vfd, V4L2_CID_MPEG_VIDEO_BITRATE,
                          (int32_t)c->enc_bitrate);
        if (r < 0) {
            mtkvcp_log("enc BITRATE=%u refused errno=%d", c->enc_bitrate,
                       -r);
            failed = 1;
        } else {
            c->enc_applied_bitrate = c->enc_bitrate;
        }
    }
    if (c->enc_gop != c->enc_applied_gop) {
        r = mtkvcp_s_ctrl(c->vfd, V4L2_CID_MPEG_VIDEO_GOP_SIZE,
                          (int32_t)c->enc_gop);
        if (r < 0) {
            mtkvcp_log("enc GOP_SIZE=%u refused errno=%d", c->enc_gop, -r);
            failed = 1;
        } else if (c->cap_fourcc == V4L2_PIX_FMT_H264) {
            /* 0 means "IDR every frame" on this firmware; keep the two
             * controls agreeing so an unspecified value cannot turn into
             * an all-intra stream (F13). */
            r = mtkvcp_s_ctrl(c->vfd, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD,
                              (int32_t)c->enc_gop);
            if (r < 0)
                mtkvcp_log("enc H264_I_PERIOD=%u refused errno=%d",
                           c->enc_gop, -r);
            c->enc_applied_gop = c->enc_gop;
        } else {
            c->enc_applied_gop = c->enc_gop;
        }
    }
    if (fps_num != c->enc_applied_fps_num ||
        fps_den != c->enc_applied_fps_den) {
        memset(&parm, 0, sizeof(parm));
        parm.type = ENC_OUT_TYPE;
        parm.parm.output.timeperframe.numerator = fps_den;
        parm.parm.output.timeperframe.denominator = fps_num;
        r = mtkvcp_ioctl(c->vfd, VIDIOC_S_PARM, &parm);
        if (r < 0) {
            mtkvcp_log("enc S_PARM %u/%u refused errno=%d", fps_num,
                       fps_den, -r);
            failed = 1;
        } else {
            c->enc_applied_fps_num = fps_num;
            c->enc_applied_fps_den = fps_den;
        }
    }
    c->enc_params_dirty = 0;
    return failed ? VA_STATUS_ERROR_OPERATION_FAILED : VA_STATUS_SUCCESS;
}

VAStatus mtkvcp_enc_render(struct mtkvcp_drv *d, int ci, int bi)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    struct mtkvcp_buffer *b = &d->buffers[bi];
    if (c->enc_target < 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    switch (b->type) {
    case VAEncSequenceParameterBufferType: {
        /* H264 and HEVC seq params both lead with the fields we
         * need at compatible offsets; distinguish by codec. */
        if (c->cap_fourcc == V4L2_PIX_FMT_H264) {
            VAEncSequenceParameterBufferH264 *s =
                (VAEncSequenceParameterBufferH264 *)b->data;
            if (s->intra_period) {
                c->enc_gop = s->intra_period;
                c->enc_params_dirty = 1;
            }
            if (s->bits_per_second) {
                c->enc_bitrate = s->bits_per_second;
                c->enc_bitrate_explicit = 1;
                c->enc_params_dirty = 1;
            }
        } else {
            VAEncSequenceParameterBufferHEVC *s =
                (VAEncSequenceParameterBufferHEVC *)b->data;
            if (s->intra_period) {
                c->enc_gop = s->intra_period;
                c->enc_params_dirty = 1;
            }
            if (s->bits_per_second) {
                c->enc_bitrate = s->bits_per_second;
                c->enc_bitrate_explicit = 1;
                c->enc_params_dirty = 1;
            }
        }
        c->enc_seq_written = 1;
        break;
    }
    case VAEncMiscParameterBufferType: {
        VAEncMiscParameterBuffer *m = (VAEncMiscParameterBuffer *)b->data;
        if (m->type == VAEncMiscParameterTypeFrameRate) {
            VAEncMiscParameterFrameRate *f =
                (VAEncMiscParameterFrameRate *)m->data;
            if (f->framerate & 0xffff) {
                c->enc_framerate_num = f->framerate & 0xffff;
                c->enc_framerate_den = (f->framerate >> 16) & 0xffff;
                if (!c->enc_framerate_den)
                    c->enc_framerate_den = 1;
                /* KPipeWire's 1000/1 is a timestamp time base, not a
                 * frame rate. Separately, this firmware accepts 119/1 but
                 * stops completing frames at 120/1 and 121/1 even for
                 * 1280x720 input. Keep the real client frame pacing in
                 * Sunshine, but send the highest verified working rate to
                 * V4L2 so a high-refresh request cannot wedge the session. */
                if (c->enc_framerate_num >
                    240u * c->enc_framerate_den) {
                    mtkvcp_log("enc framerate %u/%u implausible, using 30/1",
                               c->enc_framerate_num, c->enc_framerate_den);
                    c->enc_framerate_num = 30;
                    c->enc_framerate_den = 1;
                } else if (c->enc_framerate_num >=
                           120u * c->enc_framerate_den) {
                    if (c->enc_applied_fps_num != 119 ||
                        c->enc_applied_fps_den != 1)
                        mtkvcp_log("enc framerate %u/%u wedges firmware, using 119/1",
                                   c->enc_framerate_num,
                                   c->enc_framerate_den);
                    c->enc_framerate_num = 119;
                    c->enc_framerate_den = 1;
                }
                c->enc_params_dirty = 1;
            }
        } else if (m->type == VAEncMiscParameterTypeRateControl) {
            VAEncMiscParameterRateControl *rc =
                (VAEncMiscParameterRateControl *)m->data;
            if (rc->bits_per_second) {
                c->enc_bitrate = rc->bits_per_second;
                c->enc_bitrate_explicit = 1;
                c->enc_params_dirty = 1;
            }
            /* Quality-only clients (KPipeWire sets global_quality and no
             * bitrate) land here with an ICQ/quality factor. The firmware
             * has no QP mode, so remember the factor and let
             * mtkvcp_enc_apply_params turn it into a CBR target. */
            if (rc->ICQ_quality_factor) {
                c->enc_quality_factor = rc->ICQ_quality_factor;
                c->enc_params_dirty = 1;
            } else if (rc->quality_factor) {
                c->enc_quality_factor = rc->quality_factor;
                c->enc_params_dirty = 1;
            }
        }
        break;
    }
    case VAEncPictureParameterBufferType: {
        /* coded_buf sits inside the codec picture params. */
        size_t off = c->cap_fourcc == V4L2_PIX_FMT_H264 ?
            offsetof(VAEncPictureParameterBufferH264, coded_buf) :
            offsetof(VAEncPictureParameterBufferHEVC, coded_buf);
        VABufferID coded;
        int cbi;
        if ((size_t)b->size * (size_t)b->num_elements < off + sizeof(coded))
            return VA_STATUS_ERROR_INVALID_BUFFER;
        memcpy(&coded, (uint8_t *)b->data + off, sizeof(coded));
        /* In CQP mode ffmpeg sends no rate-control misc buffer: the
         * quality it wants is the QP carried here. pic_init_qp is the
         * H.264 and HEVC name for the same field at the same offset
         * after coded_buf, so one read covers both codecs. */
        if (!c->enc_bitrate_explicit && !c->enc_quality_factor) {
            uint8_t qp;
            size_t qoff = c->cap_fourcc == V4L2_PIX_FMT_H264 ?
                offsetof(VAEncPictureParameterBufferH264, pic_init_qp) :
                offsetof(VAEncPictureParameterBufferHEVC, pic_init_qp);
            if ((size_t)b->size * (size_t)b->num_elements >=
                qoff + sizeof(qp)) {
                memcpy(&qp, (uint8_t *)b->data + qoff, sizeof(qp));
                if (qp >= 1 && qp <= 51) {
                    c->enc_quality_factor = qp;
                    c->enc_params_dirty = 1;
                }
            }
        }
        cbi = (int)coded - 1;
        if (cbi < 0 || cbi >= MTKVCP_MAX_BUFFERS ||
            !d->buffers[cbi].in_use ||
            d->buffers[cbi].type != VAEncCodedBufferType)
            return VA_STATUS_ERROR_INVALID_BUFFER;
        c->enc_coded_buf = cbi;
        break;
    }
    case VAEncSliceParameterBufferType: {
        /* A client I-slice means "start a fresh GOP here". */
        unsigned int n = b->num_elements, k;
        if (c->cap_fourcc == V4L2_PIX_FMT_H264) {
            VAEncSliceParameterBufferH264 *sp =
                (VAEncSliceParameterBufferH264 *)b->data;
            for (k = 0; k < n; k++)
                if ((sp[k].slice_type % 5) == 2)
                    c->enc_force_idr = 1;
        } else {
            VAEncSliceParameterBufferHEVC *sp =
                (VAEncSliceParameterBufferHEVC *)b->data;
            for (k = 0; k < n; k++)
                if (sp[k].slice_type == 2)
                    c->enc_force_idr = 1;
        }
        break;
    }
    case VAEncPackedHeaderParameterBufferType:
    case VAEncPackedHeaderDataBufferType:
        /* Firmware emits its own headers. */
        break;
    default:
        break;
    }
    return VA_STATUS_SUCCESS;
}

/* Copy NV12 into the OUTPUT buffer. Source rows (sh) may be fewer
 * than destination rows (dh); the padding tail is zeroed like the
 * driver's own staging. */
static void mtkvcp_copy_nv12(uint8_t *dst, int dst_stride, int dh,
                             const uint8_t *src, int src_stride, int sh,
                             int sw, int w)
{
    int y, rows = sh < dh ? sh : dh;
    const uint8_t *sp = src;
    uint8_t *dp = dst;
    /* Never read past the source row: an imported 2460-wide frame has
     * src_stride 9856 while the context is 2464 wide, and copying the
     * context width would run into the next row's padding. */
    if (sw < w)
        w = sw;
    for (y = 0; y < rows; y++, sp += src_stride, dp += dst_stride)
        memcpy(dp, sp, (size_t)w);
    for (; y < dh; y++, dp += dst_stride)
        memset(dp, 0, (size_t)w);
    sp = src + (size_t)src_stride * (size_t)rows;
    dp = dst + (size_t)dst_stride * (size_t)dh;
    rows = (sh < dh ? sh : dh) / 2;
    for (y = 0; y < rows; y++, sp += src_stride, dp += dst_stride)
        memcpy(dp, sp, (size_t)w);
    for (; y < dh / 2; y++, dp += dst_stride)
        memset(dp, 0, (size_t)w);
}

/* Like mtkvcp_copy_nv12 but with an explicit source chroma offset: buffers
 * exported from the decoder node pitch their UV plane after the aligned
 * buffer height, while the firmware reads it after the visible height. */
static void mtkvcp_copy_nv12_uv(uint8_t *dst, int dst_stride, int dh,
                                const uint8_t *src, int src_stride,
                                size_t src_uv_off, int sh, int sw, int w)
{
    int y, rows = sh < dh ? sh : dh;
    const uint8_t *sp = src;
    uint8_t *dp = dst;

    if (sw < w)
        w = sw;
    for (y = 0; y < rows; y++, sp += src_stride, dp += dst_stride)
        memcpy(dp, sp, (size_t)w);
    for (; y < dh; y++, dp += dst_stride)
        memset(dp, 0, (size_t)w);
    sp = src + src_uv_off;
    dp = dst + (size_t)dst_stride * (size_t)dh;
    rows = (sh < dh ? sh : dh) / 2;
    for (y = 0; y < rows; y++, sp += src_stride, dp += dst_stride)
        memcpy(dp, sp, (size_t)w);
    for (; y < dh / 2; y++, dp += dst_stride)
        memset(dp, 0, (size_t)w);
}

/* Copy one packed 32-bit RGB frame into the OUTPUT buffer. The rows are
 * already in the layout the firmware reads (both strides are four bytes
 * per sample, rows pitched to the aligned width), so this is a row copy
 * with zero padding past the source rows and the visible width. Only
 * visible pixels are touched: everything the firmware does not read
 * stays as the driver left it, exactly like the NV12 path.
 */
static void mtkvcp_copy_rgb(uint8_t *dst, int dst_stride, int dh,
                            const uint8_t *src, int src_stride, int sh,
                            int sw, int w)
{
    int y, rows = sh < dh ? sh : dh;
    int bytes = (sw < w ? sw : w) * 4;
    const uint8_t *sp = src;
    uint8_t *dp = dst;
    for (y = 0; y < rows; y++, sp += src_stride, dp += dst_stride)
        memcpy(dp, sp, (size_t)bytes);
    for (; y < dh; y++, dp += dst_stride)
        memset(dp, 0, (size_t)bytes);
}

static VAStatus mtkvcp_enc_stream_start(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    VAStatus st = mtkvcp_claim_hw(d, ci);
    if (st != VA_STATUS_SUCCESS)
        return st;
    st = mtkvcp_enc_apply_params(c);
    if (st != VA_STATUS_SUCCESS)
        mtkvcp_log("enc params partially applied at stream start");
    if (mtkvcp_v4l2_stream(c->vfd, ENC_OUT_TYPE, 1) < 0 ||
        mtkvcp_v4l2_stream(c->vfd, ENC_CAP_TYPE, 1) < 0) {
        int e = errno;
        mtkvcp_v4l2_stream(c->vfd, ENC_OUT_TYPE, 0);
        mtkvcp_v4l2_stream(c->vfd, ENC_CAP_TYPE, 0);
        mtkvcp_release_hw(d, ci);
        return e == EBUSY ? VA_STATUS_ERROR_HW_BUSY :
                            VA_STATUS_ERROR_OPERATION_FAILED;
    }
    c->streaming = 1;
    return VA_STATUS_SUCCESS;
}

#ifndef VA_STATUS_ERROR_TIMEDOUT
#define VA_STATUS_ERROR_TIMEDOUT VA_STATUS_ERROR_OPERATION_FAILED
#endif

/* ---- asynchronous completion pump (F1/F2/F10/F11) ----
 *
 * EndPicture submits and returns; OUTPUT and CAPTURE completions are
 * consumed here as independent events and matched to jobs by the OUTPUT
 * timestamp cookie. The CAPTURE payload is copied out before the buffer is
 * handed back to the driver (F10), every flag/offset is validated first
 * (F14), and events/errors are drained so poll can never spin (F11).
 */
static struct mtkvcp_enc_job *mtkvcp_enc_job_by_seq(struct mtkvcp_context *c,
                                                    uint64_t seq)
{
    int i;

    for (i = 0; i < MTKVCP_ENC_MAX_JOBS; i++)
        if (c->enc_jobs[i].in_use && c->enc_jobs[i].seq == seq)
            return &c->enc_jobs[i];
    return NULL;
}

static void mtkvcp_enc_job_retire(struct mtkvcp_drv *d,
                                  struct mtkvcp_context *c,
                                  struct mtkvcp_enc_job *job)
{
    if (!job->output_done || !job->coded_done)
        return;
    if (job->alias_fd >= 0)
        close(job->alias_fd);
    if (job->surface >= 0 && job->surface < MTKVCP_MAX_SURFACES &&
        d->surfaces[job->surface].enc_busy > 0)
        d->surfaces[job->surface].enc_busy--;
    job->surface = -1;
    job->in_use = 0;
    if (c->enc_outstanding > 0)
        c->enc_outstanding--;
}

static void mtkvcp_enc_job_finish_coded(struct mtkvcp_drv *d,
                                        struct mtkvcp_context *c,
                                        struct mtkvcp_enc_job *job,
                                        const void *src, size_t got,
                                        int failed)
{
    struct mtkvcp_buffer *cb = NULL;
    VACodedBufferSegment *seg;

    if (job->coded_buf >= 0 && job->coded_buf < MTKVCP_MAX_BUFFERS &&
        d->buffers[job->coded_buf].in_use &&
        d->buffers[job->coded_buf].is_coded_seg)
        cb = &d->buffers[job->coded_buf];
    if (cb) {
        if (!failed && src && got <= cb->coded_capacity) {
            memcpy(cb->coded_bytes, src, got);
            cb->coded_size = got;
            cb->coded_status = 0; /* VA_CODED_STATUS_NO_ERROR */
        } else {
            cb->coded_size = 0;
            cb->coded_status = 1;
            failed = 1;
        }
        seg = (VACodedBufferSegment *)cb->data;
        memset(seg, 0, sizeof(*seg));
        seg->size = (uint32_t)cb->coded_size;
        seg->bit_offset = 0;
        seg->status = cb->coded_status;
        seg->buf = cb->coded_bytes;
        seg->next = NULL;
        cb->coded_busy = 0;
    }
    job->status = failed ? VA_STATUS_ERROR_ENCODING_ERROR : 0;
    job->coded_done = 1;
    mtkvcp_enc_job_retire(d, c, job);
}

void mtkvcp_enc_pump(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    int i, r;

    if (c->vfd < 0 || !c->streaming)
        return;

    /* Events first: EOS/POLLPRI must be consumed or poll spins (F11). */
    for (;;) {
        struct v4l2_event ev;

        memset(&ev, 0, sizeof(ev));
        if (mtkvcp_ioctl(c->vfd, VIDIOC_DQEVENT, &ev) < 0)
            break;
        if (ev.type == V4L2_EVENT_EOS)
            c->saw_last = 1;
    }

    /* OUTPUT returns: slot usable again and input ownership released. */
    for (;;) {
        struct v4l2_buffer b;
        struct v4l2_plane pl;

        memset(&b, 0, sizeof(b));
        memset(&pl, 0, sizeof(pl));
        b.type = ENC_OUT_TYPE;
        b.memory = c->out_dmabuf ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP;
        b.length = 1;
        b.m.planes = &pl;
        r = mtkvcp_ioctl(c->vfd, VIDIOC_DQBUF, &b);
        if (r == -EAGAIN)
            break;
        if (r < 0) {
            mtkvcp_log("enc OUTPUT DQBUF failed errno=%d", -r);
            c->error = 1;
            break;
        }
        if (b.index >= (uint32_t)c->out_count) {
            c->error = 1;
            break;
        }
        c->out_free[b.index] = 1;
        for (i = 0; i < MTKVCP_ENC_MAX_JOBS; i++) {
            struct mtkvcp_enc_job *job = &c->enc_jobs[i];

            if (job->in_use && !job->output_done &&
                job->out_slot == (int)b.index) {
                job->output_done = 1;
                mtkvcp_enc_job_retire(d, c, job);
            }
        }
    }

    /* CAPTURE returns: validate, copy, then recycle (never before). */
    for (;;) {
        struct v4l2_buffer b;
        struct v4l2_plane pl;
        struct mtkvcp_enc_job *job;
        uint64_t seq, t_recycle;
        size_t got = 0, off = 0;
        const uint8_t *src = NULL;
        int failed = 0;

        memset(&b, 0, sizeof(b));
        memset(&pl, 0, sizeof(pl));
        b.type = ENC_CAP_TYPE;
        b.memory = V4L2_MEMORY_MMAP;
        b.length = 1;
        b.m.planes = &pl;
        r = mtkvcp_ioctl(c->vfd, VIDIOC_DQBUF, &b);
        if (r == -EAGAIN)
            break;
        if (r < 0) {
            mtkvcp_log("enc CAPTURE DQBUF failed errno=%d", -r);
            c->error = 1;
            break;
        }
        seq = (uint64_t)b.timestamp.tv_sec * 1000000ull +
              (uint64_t)b.timestamp.tv_usec;
        job = mtkvcp_enc_job_by_seq(c, seq);
        if (!job)
            mtkvcp_log("enc completion for unknown seq=%llu", seq);

        got = pl.bytesused;
        off = pl.data_offset;
        if (b.index >= (uint32_t)c->cap_count || !c->enccap_map[b.index] ||
            (b.flags & V4L2_BUF_FLAG_ERROR) || off > got ||
            got - off > c->enccap_len[b.index])
            failed = 1;
        else
            src = (const uint8_t *)c->enccap_map[b.index] + off;

        {
            static int cap_dbg;

            if (cap_dbg < 12) {
                cap_dbg++;
                mtkvcp_log("enc CAP idx=%u used=%zu off=%zu len=%zu key=%d "
                           "err=%d first=%02x%02x%02x%02x%02x%02x",
                           b.index, got, off,
                           b.index < (uint32_t)c->cap_count ?
                               c->enccap_len[b.index] : 0,
                           !!(b.flags & V4L2_BUF_FLAG_KEYFRAME),
                           !!(b.flags & V4L2_BUF_FLAG_ERROR),
                           src ? src[0] : 0, src ? src[1] : 0,
                           src ? src[2] : 0, src ? src[3] : 0,
                           src ? src[4] : 0, src ? src[5] : 0);
            }
        }

        if (job) {
            uint64_t now = mtkvcp_now_ms();

            mtkvcp_enc_job_finish_coded(d, c, job, src,
                                        src ? got - off : 0, failed);
            /* frame-rate accounting at completion, once a second */
            {
                uint64_t wait_us = (now - job->submit_ms) * 1000ull;

                c->enc_frames++;
                c->enc_last_ms = now;
                c->enc_wait_us_total += wait_us;
                if (wait_us > c->enc_wait_us_max)
                    c->enc_wait_us_max = wait_us;
                if (!c->enc_last_report_ms)
                    c->enc_last_report_ms = now;
                else if (now - c->enc_last_report_ms >= 1000) {
                    uint64_t span = now - c->enc_last_report_ms;
                    uint64_t n = c->enc_frames;

                    mtkvcp_log("enc fps=%.1f frames=%llu avg_wait=%.1fms "
                               "max_wait=%.1fms coded=%zu outstanding=%d",
                               (double)n * 1000.0 / (double)span,
                               (unsigned long long)n,
                               (double)c->enc_wait_us_total / 1000.0 /
                                   (double)n,
                               (double)c->enc_wait_us_max / 1000.0,
                               got - off, c->enc_outstanding);
                    c->enc_frames = 0;
                    c->enc_wait_us_total = 0;
                    c->enc_wait_us_max = 0;
                    c->enc_last_report_ms = now;
                }
            }
            mtkvcp_prof_stage(c, MTKVCP_PROF_ENC_WAIT,
                              (mtkvcp_now_ms() - job->submit_ms) * 1000ull);
            mtkvcp_prof_frame(c, "enc");
        }

        /* Recycle only after the payload is safely copied (F10). */
        t_recycle = mtkvcp_now_us();
        {
            struct v4l2_buffer qb;
            struct v4l2_plane qp;

            memset(&qb, 0, sizeof(qb));
            memset(&qp, 0, sizeof(qp));
            qb.type = ENC_CAP_TYPE;
            qb.memory = V4L2_MEMORY_MMAP;
            qb.index = b.index;
            qb.length = 1;
            qb.m.planes = &qp;
            if (mtkvcp_ioctl(c->vfd, VIDIOC_QBUF, &qb) < 0) {
                mtkvcp_log("enc CAPTURE QBUF failed errno=%d index=%u",
                           errno, b.index);
                c->error = 1;
            } else if (b.index < (uint32_t)c->cap_count) {
                c->enccap_free[b.index] = 1;
            }
        }
        mtkvcp_prof_stage(c, MTKVCP_PROF_ENC_RECYCLE,
                          mtkvcp_now_us() - t_recycle);
    }
}

/* Wait (without holding d->lock) until the job's coded bytes are published. */
VAStatus mtkvcp_enc_wait_job(struct mtkvcp_drv *d, int ci, uint64_t seq,
                             uint64_t timeout_ns)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    uint64_t start = mtkvcp_now_us();
    uint64_t budget = timeout_ns / 1000ull;
    int infinite = timeout_ns == (uint64_t)-1;

    /* Non-blocking drain first so timeout 0 is a true query. */
    mtkvcp_enc_pump(d, ci);

    for (;;) {
        struct mtkvcp_enc_job *job = mtkvcp_enc_job_by_seq(c, seq);

        if (!job)
            return VA_STATUS_SUCCESS; /* already retired */
        if (job->coded_done)
            return job->status ? job->status : VA_STATUS_SUCCESS;
        if (c->error) {
            mtkvcp_enc_job_finish_coded(d, c, job, NULL, 0, 1);
            return VA_STATUS_ERROR_ENCODING_ERROR;
        }
        {
            struct pollfd pfd;
            uint64_t elapsed = mtkvcp_now_us() - start;
            int ms = 100;

            if (!infinite &&
                elapsed + (uint64_t)ms * 1000ull > budget) {
                int left = (int)((budget - elapsed) / 1000ull);

                if (left <= 0)
                    return VA_STATUS_ERROR_TIMEDOUT;
                ms = left;
            }
            pfd.fd = c->vfd;
            pfd.events = POLLIN | POLLPRI;
            pfd.revents = 0;
            /* Release the driver lock while blocked: VPP, other contexts
             * and Map/Sync calls on finished buffers keep moving (F2). */
            pthread_mutex_unlock(&d->lock);
            poll(&pfd, 1, ms);
            pthread_mutex_lock(&d->lock);
        }
        mtkvcp_enc_pump(d, ci);
    }
}

VAStatus mtkvcp_enc_wait_buffer(struct mtkvcp_drv *d, int bi,
                                uint64_t timeout_ns)
{
    int ci;

    for (ci = 0; ci < MTKVCP_MAX_CONTEXTS; ci++) {
        struct mtkvcp_context *c = &d->contexts[ci];
        int i;

        if (!c->in_use || !c->is_encode)
            continue;
        for (i = 0; i < MTKVCP_ENC_MAX_JOBS; i++) {
            struct mtkvcp_enc_job *job = &c->enc_jobs[i];

            if (job->in_use && job->coded_buf == bi && !job->coded_done)
                return mtkvcp_enc_wait_job(d, ci, job->seq, timeout_ns);
        }
    }
    return VA_STATUS_SUCCESS;
}

int mtkvcp_enc_buffer_busy(struct mtkvcp_drv *d, int bi)
{
    int ci;

    for (ci = 0; ci < MTKVCP_MAX_CONTEXTS; ci++) {
        struct mtkvcp_context *c = &d->contexts[ci];
        int i;

        if (!c->in_use || !c->is_encode)
            continue;
        for (i = 0; i < MTKVCP_ENC_MAX_JOBS; i++)
            if (c->enc_jobs[i].in_use && c->enc_jobs[i].coded_buf == bi)
                return 1;
    }
    return 0;
}

int mtkvcp_enc_surface_busy(struct mtkvcp_drv *d, int si)
{
    return si >= 0 && si < MTKVCP_MAX_SURFACES &&
           d->surfaces[si].enc_busy > 0;
}

VAStatus mtkvcp_enc_sync_surface(struct mtkvcp_drv *d, int si,
                                 uint64_t timeout_ns)
{
    uint64_t start = mtkvcp_now_us();
    int infinite = timeout_ns == (uint64_t)-1;

    while (mtkvcp_enc_surface_busy(d, si)) {
        int ci, waited = 0;

        if (!infinite &&
            (mtkvcp_now_us() - start) >= timeout_ns / 1000ull)
            return VA_STATUS_ERROR_TIMEDOUT;
        for (ci = 0; ci < MTKVCP_MAX_CONTEXTS; ci++) {
            struct mtkvcp_context *c = &d->contexts[ci];

            if (c->in_use && c->is_encode) {
                mtkvcp_enc_pump(d, ci);
                waited = 1;
            }
        }
        if (!waited)
            return VA_STATUS_SUCCESS;
        {
            struct pollfd pfd = { .fd = -1, .events = 0 };

            /* No single fd owns the wait; a short sleep keeps this rare
             * path simple while the pump above does the real work. */
            pthread_mutex_unlock(&d->lock);
            poll(&pfd, 0, 2);
            pthread_mutex_lock(&d->lock);
        }
    }
    return VA_STATUS_SUCCESS;
}

/* Free OUTPUT slot, preferring one that last carried the same dma-buf so
 * vb2 can keep its attachment (F6). */
static int mtkvcp_enc_pick_slot(struct mtkvcp_context *c,
                                unsigned long long key)
{
    int i, any = -1;

    for (i = 0; i < c->out_count; i++) {
        if (!c->out_free[i])
            continue;
        if (key && c->out_slot_key[i] == key)
            return i;
        if (any < 0)
            any = i;
    }
    return any;
}

/* Per-slot dma-heap staging for DMABUF-ring frames without an alias (F12):
 * the ring is a fixed memory mode, so a CPU-filled frame still needs a
 * dma-buf of its own. Allocate once per slot, refill every use. */
static int mtkvcp_enc_stage_buffer(struct mtkvcp_context *c, int slot,
                                   size_t need)
{
    if (c->out_stage_fd[slot] >= 0 && c->out_stage_len[slot] >= need)
        return c->out_stage_fd[slot];
    if (c->out_stage_map[slot]) {
        munmap(c->out_stage_map[slot], c->out_stage_len[slot]);
        c->out_stage_map[slot] = NULL;
    }
    if (c->out_stage_fd[slot] >= 0) {
        close(c->out_stage_fd[slot]);
        c->out_stage_fd[slot] = -1;
    }
    c->out_stage_fd[slot] = mtkvcp_dma_heap_alloc(need);
    if (c->out_stage_fd[slot] < 0)
        return -1;
    c->out_stage_len[slot] = need;
    c->out_stage_map[slot] = mmap(NULL, need, PROT_READ | PROT_WRITE,
                                  MAP_SHARED, c->out_stage_fd[slot], 0);
    if (c->out_stage_map[slot] == MAP_FAILED) {
        c->out_stage_map[slot] = NULL;
        close(c->out_stage_fd[slot]);
        c->out_stage_fd[slot] = -1;
        return -1;
    }
    return c->out_stage_fd[slot];
}

VAStatus mtkvcp_enc_end(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    struct mtkvcp_surface *s;
    struct mtkvcp_buffer *cb;
    struct mtkvcp_enc_job *job = NULL;
    struct v4l2_buffer b;
    struct v4l2_plane pl;
    unsigned long long key = 0;
    int slot = -1, i, r, use_alias = 0;
    uint64_t t_frame, t_stage, t_wait0;
    VAStatus st;
    if (c->enc_target < 0 || c->enc_coded_buf < 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (c->error) {
        c->enc_target = -1;
        c->enc_coded_buf = -1;
        return VA_STATUS_ERROR_ENCODING_ERROR;
    }
    s = &d->surfaces[c->enc_target];
    cb = &d->buffers[c->enc_coded_buf];
    /* A coded buffer may still belong to an earlier job: wait for it
     * instead of overwriting live bytes (F1/F14). */
    if (mtkvcp_enc_buffer_busy(d, c->enc_coded_buf)) {
        st = mtkvcp_enc_wait_buffer(d, c->enc_coded_buf,
                                    (uint64_t)ENC_TIMEOUT_MS * 1000000ull);
        if (st != VA_STATUS_SUCCESS) {
            c->enc_target = -1;
            c->enc_coded_buf = -1;
            return st;
        }
    }
    if (!cb->is_coded_seg) {
        size_t total = (size_t)cb->size * (size_t)cb->num_elements;
        void *bytes;

        if (total < sizeof(VACodedBufferSegment)) {
            c->enc_target = -1;
            c->enc_coded_buf = -1;
            return VA_STATUS_ERROR_INVALID_BUFFER;
        }
        /* The client size is encode capacity; the segment header is
         * metadata on top. Allocate first and only claim the buffer when
         * the payload storage really exists (F14). */
        bytes = malloc(total ? total : 1);
        if (!bytes) {
            c->enc_target = -1;
            c->enc_coded_buf = -1;
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        cb->coded_bytes = bytes;
        cb->coded_capacity = total;
        cb->coded_size = 0;
        cb->coded_status = 0;
        cb->is_coded_seg = 1;
    }
    cb->coded_busy = 1;
    if (!c->streaming) {
        st = mtkvcp_enc_stream_start(d, ci);
        if (st != VA_STATUS_SUCCESS) {
            cb->coded_busy = 0;
            c->enc_target = -1;
            c->enc_coded_buf = -1;
            return st;
        }
    }
    if (c->enc_params_dirty) {
        st = mtkvcp_enc_apply_params(c);
        if (st != VA_STATUS_SUCCESS)
            mtkvcp_log("enc dynamic params partially applied");
    }
    if (c->enc_force_idr) {
        r = mtkvcp_s_ctrl(c->vfd, V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 1);
        if (r < 0)
            mtkvcp_log("enc FORCE_KEY_FRAME refused errno=%d", -r);
    }
    t_frame = mtkvcp_now_us();
    t_wait0 = t_frame;
    key = s->alias_fd >= 0 ? s->alias_key : 0;
    for (;;) {
        t_stage = mtkvcp_now_us();
        mtkvcp_enc_pump(d, ci);
        slot = mtkvcp_enc_pick_slot(c, key);
        for (i = 0; i < MTKVCP_ENC_MAX_JOBS; i++)
            if (!c->enc_jobs[i].in_use) {
                job = &c->enc_jobs[i];
                break;
            }
        mtkvcp_prof_stage(c, MTKVCP_PROF_ENC_REAP,
                          mtkvcp_now_us() - t_stage);
        if (slot >= 0 && job)
            break;
        job = NULL;
        if (mtkvcp_now_us() - t_wait0 >
            (uint64_t)ENC_TIMEOUT_MS * 1000ull) {
            cb->coded_busy = 0;
            c->enc_target = -1;
            c->enc_coded_buf = -1;
            return VA_STATUS_ERROR_HW_BUSY;
        }
        {
            struct pollfd pfd = { .fd = c->vfd,
                                  .events = POLLIN | POLLPRI };

            /* Bounded backpressure while a slot/job frees: the driver
             * lock is released so VPP and other contexts keep moving. */
            pthread_mutex_unlock(&d->lock);
            poll(&pfd, 1, 20);
            pthread_mutex_lock(&d->lock);
        }
        if (c->error) {
            cb->coded_busy = 0;
            c->enc_target = -1;
            c->enc_coded_buf = -1;
            return VA_STATUS_ERROR_ENCODING_ERROR;
        }
    }
    t_stage = mtkvcp_now_us();
    memset(&b, 0, sizeof(b));
    memset(&pl, 0, sizeof(pl));
    b.type = ENC_OUT_TYPE;
    b.index = (uint32_t)slot;
    b.length = 1;
    b.m.planes = &pl;
    if (c->out_dmabuf && s->alias_fd >= 0) {
        size_t want = c->rgb_in ?
            (size_t)s->alias_stride * (size_t)c->height :
            (size_t)s->alias_stride * (size_t)c->height * 3u / 2u;

        /* Validate the alias against the negotiated layout before handing
         * the compositor's object to the driver (F12). */
        if ((size_t)s->alias_stride == (size_t)c->out_stride &&
            want <= (size_t)s->alias_size) {
            use_alias = 1;
            b.memory = V4L2_MEMORY_DMABUF;
            pl.m.fd = s->alias_fd;
            pl.length = (uint32_t)s->alias_size;
            pl.bytesused = (uint32_t)want;
        } else {
            mtkvcp_log("enc alias rejected: stride=%d size=%d want=%zu "
                       "negotiated_stride=%d", s->alias_stride,
                       s->alias_size, want, c->out_stride);
            /* The alias is this frame's only content; falling back to
             * possibly-stale CPU staging would encode the wrong picture. */
            cb->coded_busy = 0;
            c->enc_target = -1;
            c->enc_coded_buf = -1;
            return VA_STATUS_ERROR_INVALID_SURFACE;
        }
    }
    if (!use_alias) {
        size_t want = c->rgb_in ?
            (size_t)c->out_stride * (size_t)c->height :
            (size_t)c->out_stride * (size_t)c->height * 3u / 2u;
        uint8_t *dst;
        uint64_t tc;

        const uint8_t *srcp = NULL;
        int src_stride = 0, src_h = 0, src_w = 0, from_export = 0;
        size_t src_uv = 0;

        /* A surface the client exported in order to write into it (the
         * vaapi encoder of FFmpeg renders its input there through EGL)
         * carries the frame in the exported dma-buf, not in enc_data.
         * Map it once and read the pixels from there. */
        if (!c->rgb_in && s->prime_fd >= 0 &&
            s->fourcc == V4L2_PIX_FMT_NV12 && s->export_stride > 0) {
            if (!s->imp_map && s->imp_size) {
                void *m = mmap(NULL, s->imp_size, PROT_READ, MAP_SHARED,
                               s->prime_fd, 0);

                if (m == MAP_FAILED) {
                    cb->coded_busy = 0;
                    c->enc_target = -1;
                    c->enc_coded_buf = -1;
                    return VA_STATUS_ERROR_OPERATION_FAILED;
                }
                s->imp_map = m;
            }
            if (s->imp_map) {
                mtkvcp_dmabuf_cpu_read(s->prime_fd);
                srcp = s->imp_map;
                src_stride = s->export_stride;
                src_uv = (size_t)s->export_uvoff;
                src_h = s->height;
                src_w = s->width;
                from_export = 1;
            }
        }
        if (!from_export && (!s->enc_data || s->enc_size < want)) {
            void *nd = malloc(want ? want : 1);

            if (!nd) {
                cb->coded_busy = 0;
                c->enc_target = -1;
                c->enc_coded_buf = -1;
                return VA_STATUS_ERROR_ALLOCATION_FAILED;
            }
            free(s->enc_data);
            s->enc_data = nd;
            s->enc_size = want;
            memset(s->enc_data, 0, want ? want : 1);
            s->enc_stride = c->rgb_in ? c->out_stride : c->width;
            s->enc_rgb = c->rgb_in ? 1 : 0;
        }
        if (c->out_dmabuf) {
            /* F12: a DMABUF ring cannot fall back to MMAP, so CPU-filled
             * frames go through a reusable per-slot dma-heap buffer. */
            int fd = mtkvcp_enc_stage_buffer(c, slot, want);

            if (fd < 0) {
                cb->coded_busy = 0;
                c->enc_target = -1;
                c->enc_coded_buf = -1;
                return VA_STATUS_ERROR_ALLOCATION_FAILED;
            }
            dst = c->out_stage_map[slot];
            b.memory = V4L2_MEMORY_DMABUF;
            pl.m.fd = fd;
            pl.length = (uint32_t)c->out_stage_len[slot];
            pl.bytesused = (uint32_t)want;
        } else {
            dst = c->out_map[slot];
            b.memory = V4L2_MEMORY_MMAP;
            pl.length = (uint32_t)c->out_len[slot];
            pl.bytesused = (uint32_t)want;
        }
        tc = mtkvcp_now_us();
        if (from_export)
            mtkvcp_copy_nv12_uv(dst, c->out_stride, c->height,
                                srcp, src_stride, src_uv,
                                src_h, src_w, c->width);
        else if (c->rgb_in)
            mtkvcp_copy_rgb(dst, c->out_stride, c->height,
                            s->enc_data, s->enc_stride, s->height,
                            s->width, c->width);
        else
            mtkvcp_copy_nv12(dst, c->out_stride, c->height,
                             s->enc_data, s->enc_stride, s->height,
                             s->width, c->width);
        {
            uint64_t du = mtkvcp_now_us() - tc;

            c->enc_copy_us_total += du;
            if (du > c->enc_copy_us_max)
                c->enc_copy_us_max = du;
            mtkvcp_prof_stage(c, MTKVCP_PROF_ENC_COPY, du);
        }
    }
    c->seq++;
    b.timestamp.tv_sec = (long)(c->seq / 1000000ull);
    b.timestamp.tv_usec = (long)(c->seq % 1000000ull);
    t_stage = mtkvcp_now_us();
    if (mtkvcp_ioctl(c->vfd, VIDIOC_QBUF, &b) < 0) {
        mtkvcp_log("enc QBUF failed errno=%d slot=%d", errno, slot);
        cb->coded_busy = 0;
        c->enc_target = -1;
        c->enc_coded_buf = -1;
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }
    mtkvcp_prof_stage(c, MTKVCP_PROF_ENC_QBUF, mtkvcp_now_us() - t_stage);
    c->out_free[slot] = 0;
    c->out_slot_key[slot] = key;

    /* Publish the job with its submission: the completion pump owns the
     * result from here and EndPicture returns without blocking (F1). The
     * job entry was reserved above, so it cannot be NULL here. */
    job->in_use = 1;
    job->seq = c->seq;
    job->surface = c->enc_target;
    job->coded_buf = c->enc_coded_buf;
    job->out_slot = slot;
    job->alias_fd = use_alias && s->alias_fd >= 0 ?
        fcntl(s->alias_fd, F_DUPFD_CLOEXEC, 0) : -1;
    job->submit_ms = mtkvcp_now_us() / 1000ull;
    job->output_done = 0;
    job->coded_done = 0;
    job->status = 0;
    s->enc_busy++;
    c->enc_outstanding++;
    if (!c->enc_first_ms)
        c->enc_first_ms = job->submit_ms;

    mtkvcp_prof_stage(c, MTKVCP_PROF_ENC_TOTAL, mtkvcp_now_us() - t_frame);
    c->enc_target = -1;
    c->enc_coded_buf = -1;
    return VA_STATUS_SUCCESS;
}

void mtkvcp_enc_destroy(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    int i;
    if (c->vfd < 0)
        return;
    if (c->streaming && !c->error) {
        /* Drain outstanding jobs, then collect. */
        struct v4l2_encoder_cmd cmd;
        uint64_t deadline = mtkvcp_now_us() + 3000000ull;

        memset(&cmd, 0, sizeof(cmd));
        cmd.cmd = V4L2_ENC_CMD_STOP;
        mtkvcp_ioctl(c->vfd, VIDIOC_ENCODER_CMD, &cmd);
        while (c->enc_outstanding && mtkvcp_now_us() < deadline) {
            struct pollfd pfd = { .fd = c->vfd,
                                  .events = POLLIN | POLLPRI };

            mtkvcp_enc_pump(d, ci);
            if (!c->enc_outstanding)
                break;
            pthread_mutex_unlock(&d->lock);
            poll(&pfd, 1, 100);
            pthread_mutex_lock(&d->lock);
        }
        /* Anything still open after the budget fails rather than leaks. */
        for (i = 0; i < MTKVCP_ENC_MAX_JOBS; i++) {
            struct mtkvcp_enc_job *job = &c->enc_jobs[i];

            if (!job->in_use)
                continue;
            if (!job->coded_done)
                mtkvcp_enc_job_finish_coded(d, c, job, NULL, 0, 1);
            if (!job->output_done) {
                job->output_done = 1;
                mtkvcp_enc_job_retire(d, c, job);
            }
        }
        mtkvcp_v4l2_stream(c->vfd, ENC_OUT_TYPE, 0);
        mtkvcp_v4l2_stream(c->vfd, ENC_CAP_TYPE, 0);
        mtkvcp_v4l2_reqbufs_mem(c->vfd, ENC_OUT_TYPE, 0,
                                c->out_dmabuf ? V4L2_MEMORY_DMABUF :
                                                V4L2_MEMORY_MMAP);
        mtkvcp_v4l2_reqbufs(c->vfd, ENC_CAP_TYPE, 0);
        c->streaming = 0;
    } else if (!c->streaming) {
        mtkvcp_v4l2_reqbufs_mem(c->vfd, ENC_OUT_TYPE, 0,
                                c->out_dmabuf ? V4L2_MEMORY_DMABUF :
                                                V4L2_MEMORY_MMAP);
        mtkvcp_v4l2_reqbufs(c->vfd, ENC_CAP_TYPE, 0);
    }
    for (i = 0; i < MTKVCP_OUT_BUFS_ENC; i++) {
        if (c->out_stage_map[i]) {
            munmap(c->out_stage_map[i], c->out_stage_len[i]);
            c->out_stage_map[i] = NULL;
        }
        if (c->out_stage_fd[i] >= 0) {
            close(c->out_stage_fd[i]);
            c->out_stage_fd[i] = -1;
        }
        if (c->out_map[i]) {
            munmap(c->out_map[i], c->out_len[i]);
            c->out_map[i] = NULL;
        }
    }
    for (i = 0; i < MTKVCP_CAP_BUFS_ENC; i++)
        if (c->enccap_map[i]) {
            munmap(c->enccap_map[i], c->enccap_len[i]);
            c->enccap_map[i] = NULL;
        }
    close(c->vfd);
    c->vfd = -1;
    mtkvcp_release_hw(d, ci);
}
