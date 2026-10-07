/* SPDX-License-Identifier: MIT */
/* Decode contexts: stateful V4L2 M2M driven one access unit per picture. */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <va/va.h>
#include <va/va_backend.h>

#include "va_mtkvcp.h"

#define DEC_OUT_TYPE V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
#define DEC_CAP_TYPE V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
#define SYNC_TIMEOUT_MS 30000

/* Vendor V4L2_CID_MPEG_MTK_OPERATION_RATE is MTK_BASE+16, the same control
 * downstream carries a client's declared decode rate in. The kernel sizes its
 * OPP request from it: the request is a pixel rate, so a stream above the
 * codec's 60/75 fps floor needs the real rate or the decoder is clocked too
 * low to keep up.
 *
 * Nothing in this stack can supply that number by itself. No VAAPI decode
 * structure carries a frame rate (num_units_in_tick/time_scale exist only in
 * the encode headers), the kernel's OUTPUT timestamp is this bridge's own
 * sequence cookie so no PTS reaches it either, and the stream's VUI is not in
 * the buffers ffmpeg hands over - it passes parsed picture parameters and raw
 * slice NALs, not whole Annex-B access units.
 *
 * So the rate is an explicit opt-in. A client that consumes frames faster than
 * the floor - a 120/144 Hz panel, or playback at more than 1x - exports
 * MTK_VCP_VA_OP_RATE=<fps> and the driver is asked for the step that needs.
 * Unset or 0 leaves the driver on its floor, which is what everything before
 * this control ran with.
 */
#define MTKVCP_DEC_OP_RATE_CID (V4L2_CTRL_CLASS_CODEC | 0x2010)

static uint32_t mtkvcp_dec_op_rate(void)
{
    const char *v = getenv("MTK_VCP_VA_OP_RATE");
    long n;

    if (!v || !*v)
        return 0;
    n = strtol(v, NULL, 10);
    if (n <= 0 || n > 1000)
        return 0;
    return (uint32_t)n;
}

/* Send the synthesised VPS/SPS/PPS only when the picture parameters change,
 * instead of with every access unit. Diagnostic: the kernel path sends the
 * stream's own sets, which appear once per sequence. */
static int mtkvcp_ps_once(void)
{
    static int cached = -1;

    if (cached < 0)
        cached = getenv("MTK_VCP_VA_PS_ONCE") ? 1 : 0;
    return cached;
}

/* Tell the driver the rate this session will actually consume frames at. The
 * control is per open file handle, so it has to be set here rather than by an
 * external v4l2-ctl. Failure is not fatal: an older kernel has no such
 * control, and the stream then runs at the codec floor exactly as before.
 */
static void mtkvcp_dec_set_op_rate(struct mtkvcp_context *c)
{
    uint32_t rate = mtkvcp_dec_op_rate();
    int r;

    if (!rate)
        return;
    r = mtkvcp_v4l2_s_ctrl(c->vfd, MTKVCP_DEC_OP_RATE_CID, (int32_t)rate);
    if (r < 0)
        fprintf(stderr, "mtk-vcp-va: operation rate %u not accepted: %d "
                "(kernel without the control?)\n", rate, r);
    else
        mtkvcp_log("operation rate %u accepted", rate);
}

static uint64_t mtkvcp_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)ts.tv_nsec / 1000000ull;
}

static uint64_t mtkvcp_ts_to_seq(const struct timeval *tv)
{
    return (uint64_t)tv->tv_sec * 1000000ull + (uint64_t)tv->tv_usec;
}

static void mtkvcp_seq_to_ts(uint64_t seq, struct timeval *tv)
{
    tv->tv_sec = (long)(seq / 1000000ull);
    tv->tv_usec = (long)(seq % 1000000ull);
}

static int mtkvcp_ioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do {
        r = ioctl(fd, req, arg);
    } while (r < 0 && errno == EINTR);
    return r < 0 ? -errno : 0;
}

/* Take this context's own VCP decode session. */
static VAStatus mtkvcp_claim_hw(struct mtkvcp_drv *d, int ci)
{
    /* The kernel has allowed concurrent VCP decoder sessions since
     * "allow concurrent VCP decoder sessions", and two 720p60 streams are
     * verified running together. The exclusive owner test here predates
     * that: it made a second decode context inside one VA display fail with
     * HW_BUSY, which is what VLC does on some runs - it opens a second
     * context mid-playback, gets the failure, and thrashes. Sessions are per
     * context now; the kernel arbitrates. */
    struct mtkvcp_context *c = &d->contexts[ci];

    if (c->hw_owned)
        return VA_STATUS_SUCCESS;
    c->hw_owned = 1;
    d->hw_owner_ctx = ci;
    return VA_STATUS_SUCCESS;
}

static void mtkvcp_release_hw(struct mtkvcp_drv *d, int ci)
{
    d->contexts[ci].hw_owned = 0;
    if (d->hw_owner_ctx == ci)
        d->hw_owner_ctx = -1;
}

static void mtkvcp_unmap_all(struct mtkvcp_context *c)
{
    int i;
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++) {
        if (c->cap_export[i]) {
            close(c->cap_export[i] - 1);
            c->cap_export[i] = 0;
        }
        if (c->cap_export_uv[i]) {
            close(c->cap_export_uv[i] - 1);
            c->cap_export_uv[i] = 0;
        }
    }
    for (i = 0; i < MTKVCP_OUT_BUFS_DEC; i++)
        if (c->out_map[i]) {
            munmap(c->out_map[i], c->out_len[i]);
            c->out_map[i] = NULL;
        }
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++) {
        if (c->cap_map[i]) {
            munmap(c->cap_map[i], c->cap_len[i]);
            c->cap_map[i] = NULL;
        }
        if (c->cap_map_uv[i]) {
            munmap(c->cap_map_uv[i], c->cap_len_uv[i]);
            c->cap_map_uv[i] = NULL;
        }
    }
    free(c->tile_scratch);
    c->tile_scratch = NULL;
    c->tile_scratch_sz = 0;
}

/* QUERYBUF + mmap one CAPTURE buffer. Tiled capture has a second, separate
 * V4L2 plane holding the chroma tiles; the linear path has one.
 */
static int mtkvcp_map_cap(struct mtkvcp_context *c, int idx)
{
    struct v4l2_buffer b;
    struct v4l2_plane pl[2];
    int np = c->cap_planes > 1 ? 2 : 1;

    memset(&b, 0, sizeof(b));
    memset(pl, 0, sizeof(pl));
    b.type = DEC_CAP_TYPE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = (uint32_t)idx;
    b.length = (uint32_t)np;
    b.m.planes = pl;
    if (mtkvcp_ioctl(c->vfd, VIDIOC_QUERYBUF, &b) < 0)
        return -1;
    c->cap_len[idx] = pl[0].length;
    c->cap_map[idx] = mmap(NULL, pl[0].length, PROT_READ | PROT_WRITE,
                           MAP_SHARED, c->vfd, pl[0].m.mem_offset);
    if (c->cap_map[idx] == MAP_FAILED) {
        c->cap_map[idx] = NULL;
        return -1;
    }
    if (np < 2)
        return 0;
    c->cap_len_uv[idx] = pl[1].length;
    c->cap_map_uv[idx] = mmap(NULL, pl[1].length, PROT_READ | PROT_WRITE,
                              MAP_SHARED, c->vfd, pl[1].m.mem_offset);
    if (c->cap_map_uv[idx] == MAP_FAILED) {
        c->cap_map_uv[idx] = NULL;
        return -1;
    }
    return 0;
}

static void mtkvcp_unmap_cap(struct mtkvcp_context *c, int idx)
{
    if (c->cap_map[idx]) {
        munmap(c->cap_map[idx], c->cap_len[idx]);
        c->cap_map[idx] = NULL;
    }
    if (c->cap_map_uv[idx]) {
        munmap(c->cap_map_uv[idx], c->cap_len_uv[idx]);
        c->cap_map_uv[idx] = NULL;
    }
}

VAStatus mtkvcp_dec_create(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    const char *node = getenv("MTK_VCP_VA_DEC_NODE");
    int r, i;
    if (!node || !*node)
        node = MTKVCP_DEC_NODE;
    c->vfd = mtkvcp_v4l2_open(node);
    if (c->vfd < 0)
        return VA_STATUS_ERROR_OPERATION_FAILED;
    /* OUTPUT carries one AU per buffer. Like the validated probes,
     * offer w*h*2 so large access units always fit. */
    {
        struct v4l2_format f;
        memset(&f, 0, sizeof(f));
        f.type = DEC_OUT_TYPE;
        f.fmt.pix_mp.width = (uint32_t)c->width;
        f.fmt.pix_mp.height = (uint32_t)c->height;
        f.fmt.pix_mp.pixelformat = c->out_fourcc;
        f.fmt.pix_mp.field = V4L2_FIELD_NONE;
        f.fmt.pix_mp.num_planes = 1;
        f.fmt.pix_mp.plane_fmt[0].sizeimage =
            (uint32_t)((size_t)c->width * (size_t)c->height * 2u);
        r = mtkvcp_ioctl(c->vfd, VIDIOC_S_FMT, &f);
    }
    if (r < 0) {
        close(c->vfd);
        c->vfd = -1;
        return VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED;
    }
    r = mtkvcp_v4l2_reqbufs(c->vfd, DEC_OUT_TYPE, MTKVCP_OUT_BUFS_DEC);
    if (r < MTKVCP_OUT_BUFS_DEC) {
        close(c->vfd);
        c->vfd = -1;
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    c->mp2_N = 2;
    c->mp2_pending = 0;
    c->mp2_last_anchor = -1;
    c->mp2_anchored_once = 0;
    c->out_count = MTKVCP_OUT_BUFS_DEC;
    for (i = 0; i < c->out_count; i++) {
        struct v4l2_buffer b;
        struct v4l2_plane pl;
        memset(&b, 0, sizeof(b));
        memset(&pl, 0, sizeof(pl));
        b.type = DEC_OUT_TYPE;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = (uint32_t)i;
        b.length = 1;
        b.m.planes = &pl;
        if (mtkvcp_ioctl(c->vfd, VIDIOC_QUERYBUF, &b) < 0) {
            mtkvcp_unmap_all(c);
            close(c->vfd);
            c->vfd = -1;
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        c->out_len[i] = pl.length;
        c->out_map[i] = mmap(NULL, pl.length, PROT_READ | PROT_WRITE,
                             MAP_SHARED, c->vfd, pl.m.mem_offset);
        if (c->out_map[i] == MAP_FAILED) {
            c->out_map[i] = NULL;
            mtkvcp_unmap_all(c);
            close(c->vfd);
            c->vfd = -1;
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        c->out_free[i] = 1;
    }
    mtkvcp_v4l2_subscribe(c->vfd, V4L2_EVENT_SOURCE_CHANGE);
    mtkvcp_v4l2_subscribe(c->vfd, V4L2_EVENT_EOS);
    return VA_STATUS_SUCCESS;
}

/* Queue every CAPTURE buffer once (all bound surfaces start queued). */
static int mtkvcp_qbuf_cap(struct mtkvcp_context *c, int idx)
{
    struct v4l2_buffer b;
    struct v4l2_plane pl[2];
    int np = c->cap_planes > 1 ? 2 : 1;
    memset(&b, 0, sizeof(b));
    memset(pl, 0, sizeof(pl));
    b.type = DEC_CAP_TYPE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = (uint32_t)idx;
    b.length = (uint32_t)np;
    b.m.planes = pl;
    int r = mtkvcp_ioctl(c->vfd, VIDIOC_QBUF, &b);

    if (r == 0)
        c->cap_queued++;
    return r;
}

/* Largest CAPTURE pool worth allocating for this frame: bounded by the
 * kernel's vb2 limit and a memory budget, but always enough for the
 * baseline plus the reserve. */
/* Pool sizing knobs, so the reserve can be A/B-ed without a rebuild.
 * MTK_VCP_VA_CAP_RESERVE (default MTKVCP_CAP_RESERVE) and
 * MTK_VCP_VA_CAP_MAX (default MTKVCP_MAX_CAP_BUFS). */
static int mtkvcp_cap_reserve(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *v = getenv("MTK_VCP_VA_CAP_RESERVE");

        cached = v ? atoi(v) : MTKVCP_CAP_RESERVE;
        if (cached < 0)
            cached = 0;
    }
    return cached;
}

static int mtkvcp_cap_max(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *v = getenv("MTK_VCP_VA_CAP_MAX");

        cached = v ? atoi(v) : MTKVCP_MAX_CAP_BUFS;
        if (cached < 1)
            cached = 1;
    }
    return cached;
}

static int mtkvcp_dec_max_pool(struct mtkvcp_context *c)
{
    size_t stride = c->cap_stride > 0 ? (size_t)c->cap_stride : (size_t)c->width;
    size_t bh = c->cap_bh > 0 ? (size_t)c->cap_bh : (size_t)c->height;
    size_t frame = stride * bh * 3u / 2u;
    int max = mtkvcp_cap_max();

    if (frame) {
        int by_mem = (int)(MTKVCP_CAP_BUDGET / frame);

        if (by_mem < max)
            max = by_mem;
    }
    if (max < mtkvcp_cap_reserve() + 2)
        max = mtkvcp_cap_reserve() + 2;
    return max;
}

/* Grow the CAPTURE queue as the client binds more decode surfaces. One
 * buffer per held surface is what lets the client keep its frames, plus
 * MTKVCP_CAP_RESERVE destinations the kernel needs to keep running at
 * all. vb2 accepts CREATE_BUFS while streaming, so the pool follows the
 * client's real depth instead of sizing for the worst case up front. */
static void mtkvcp_dec_grow_capture(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    struct v4l2_create_buffers cb;
    struct v4l2_format g;
    int bound = 0, need, i;

    /* Before STREAMON the CAPTURE format is not negotiated yet and the
     * baseline pool is always enough; growing then would allocate the
     * wrong geometry. Pre-bound targets are covered by stream start. */
    if (!c->streaming)
        return;
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
        if (d->surfaces[i].in_use && d->surfaces[i].ctx == ci &&
            d->surfaces[i].kind == MTKVCP_SURF_DECODE)
            bound++;
    /* +1 for the surface about to bind. */
    need = bound + 1 + mtkvcp_cap_reserve();
    {
        int max = mtkvcp_dec_max_pool(c);

        if (need > max)
            need = max;
    }
    if (need <= c->cap_count || c->cap_count >= mtkvcp_cap_max())
        return;
    memset(&g, 0, sizeof(g));
    g.type = DEC_CAP_TYPE;
    if (mtkvcp_ioctl(c->vfd, VIDIOC_G_FMT, &g) < 0)
        return;
    memset(&cb, 0, sizeof(cb));
    cb.count = (uint32_t)(need - c->cap_count);
    cb.memory = V4L2_MEMORY_MMAP;
    cb.format = g;
    if (mtkvcp_ioctl(c->vfd, VIDIOC_CREATE_BUFS, &cb) < 0) {
        mtkvcp_log("grow CAPTURE: CREATE_BUFS +%u (have %d) failed",
                   cb.count, c->cap_count);
        return;
    }
    for (i = 0; i < (int)cb.count; i++) {
        int idx = (int)cb.index + i;

        if (idx < 0 || idx >= MTKVCP_MAX_SURFACES)
            break;
        if (mtkvcp_map_cap(c, idx) < 0)
            break;
        if (mtkvcp_qbuf_cap(c, idx) < 0)
            break;
        c->cap_count = idx + 1;
        c->cap_mmap_count = idx + 1;
    }
    mtkvcp_log("grow CAPTURE: bound=%d count=%d queued=%d", bound,
               c->cap_count, c->cap_queued);
}

/* Fixed CAPTURE pool: DPB + reorder + display slack. VAAPI clients
 * (notably ffmpeg) bind surfaces lazily one picture at a time, so the
 * pool cannot be sized by the surfaces known at stream start. Slots
 * without a client surface are driver-held spares; the completion
 * swap assigns buffers to targets as frames arrive. */
static int mtkvcp_dec_pool_size(struct mtkvcp_context *c)
{
    int big = c->width > 1920 || c->height > 1088;
    /* NOTE: raising the large-frame pool from 12 to 20 was tried and measured
     * to change nothing (2460x1080: 1200 -> 1206 frames; 2560x1600: 924 -> 941
     * over 20 s of VLC playback), so it is reverted rather than kept. The
     * 12-vs-20 split correlating with "laggy vs smooth" was a coincidence.
     */
    if (c->out_fourcc == V4L2_PIX_FMT_H264)
        return big ? 12 : 20;
    if (c->out_fourcc == V4L2_PIX_FMT_HEVC)
        return big ? 12 : 16;
    return 12;
}

/* Stream start: CAPTURE setup with the fixed pool. */
static VAStatus mtkvcp_dec_stream_start(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    int bound[MTKVCP_MAX_SURFACES];
    int nbound = 0, i, r, granted;
    size_t dummy;
    VAStatus st;
    /* Before STREAMON: the driver sizes its OPP request when the first header
     * is parsed, which happens on the first access unit after this. */
    mtkvcp_dec_set_op_rate(c);
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
        if (d->surfaces[i].in_use && d->surfaces[i].ctx == ci &&
            d->surfaces[i].kind == MTKVCP_SURF_DECODE)
            bound[nbound++] = i;
    c->pool_size = mtkvcp_dec_pool_size(c);
    /* Pre-bound render targets each need their own destination. */
    if (nbound + mtkvcp_cap_reserve() + 1 > c->pool_size)
        c->pool_size = nbound + mtkvcp_cap_reserve() + 1;
    {
        int max = mtkvcp_dec_max_pool(c);

        if (c->pool_size > max)
            c->pool_size = max;
    }
    st = mtkvcp_claim_hw(d, ci);
    if (st != VA_STATUS_SUCCESS)
        return st;
    r = mtkvcp_v4l2_s_fmt_planes(c->vfd, DEC_CAP_TYPE, c->cap_fourcc,
                                 c->width, c->height, c->cap_planes, &dummy);
    mtkvcp_log("CAP S_FMT -> sizeimage=%zu", dummy);
    if (r < 0) {
        mtkvcp_release_hw(d, ci);
        return VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED;
    }
    granted = mtkvcp_v4l2_reqbufs(c->vfd, DEC_CAP_TYPE, c->pool_size);
    if (granted < c->pool_size) {
        mtkvcp_log("stream start: REQBUFS granted %d < %d", granted,
                   c->pool_size);
        mtkvcp_v4l2_reqbufs(c->vfd, DEC_CAP_TYPE, 0);
        mtkvcp_release_hw(d, ci);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    /* Real stride/buffer-height for image export (may exceed visible). */
    {
        struct v4l2_format g;
        memset(&g, 0, sizeof(g));
        g.type = DEC_CAP_TYPE;
        if (mtkvcp_ioctl(c->vfd, VIDIOC_G_FMT, &g) == 0) {
            c->cap_stride =
                (int)g.fmt.pix_mp.plane_fmt[0].bytesperline;
            c->cap_bh = (int)g.fmt.pix_mp.height;
        }
        if (c->cap_stride <= 0)
            c->cap_stride = c->width;
        if (c->cap_bh < c->height)
            c->cap_bh = c->height;
    }
    c->cap_count = c->pool_size;
    c->cap_mmap_count = c->pool_size;
    for (i = 0; i < nbound && i < c->pool_size; i++) {
        if (d->surfaces[bound[i]].prime_fd >= 0) {
            close(d->surfaces[bound[i]].prime_fd);
            d->surfaces[bound[i]].prime_fd = -1;
        }
        d->surfaces[bound[i]].cap_index = i;
        d->surfaces[bound[i]].state = MTKVCP_SS_IDLE;
    }
    for (i = 0; i < c->pool_size; i++) {
        if (mtkvcp_map_cap(c, i) < 0) {
            mtkvcp_unmap_all(c);
            mtkvcp_v4l2_reqbufs(c->vfd, DEC_CAP_TYPE, 0);
            mtkvcp_release_hw(d, ci);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        r = mtkvcp_qbuf_cap(c, i);
        if (r < 0) {
            mtkvcp_unmap_all(c);
            mtkvcp_v4l2_reqbufs(c->vfd, DEC_CAP_TYPE, 0);
            mtkvcp_release_hw(d, ci);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }
    }
    /* The VCP is a single global session and its remoteproc stop/start
     * cycle takes seconds, so a playback started right after another one
     * (or after any other client) can see STREAMON return EBUSY for a
     * while. Wait for the hardware instead of failing the client. */
    {
        int streamed, tries = 0;

        for (;;) {
            streamed = mtkvcp_v4l2_stream(c->vfd, DEC_OUT_TYPE, 1);
            if (streamed == 0) {
                streamed = mtkvcp_v4l2_stream(c->vfd, DEC_CAP_TYPE, 1);
                if (streamed == 0)
                    break;
                mtkvcp_v4l2_stream(c->vfd, DEC_OUT_TYPE, 0);
            }
            if (streamed != -EBUSY || ++tries > 40) {
                mtkvcp_unmap_all(c);
                mtkvcp_v4l2_stream(c->vfd, DEC_OUT_TYPE, 0);
                mtkvcp_v4l2_stream(c->vfd, DEC_CAP_TYPE, 0);
                mtkvcp_v4l2_reqbufs(c->vfd, DEC_CAP_TYPE, 0);
                mtkvcp_release_hw(d, ci);
                return streamed == -EBUSY ? VA_STATUS_ERROR_HW_BUSY :
                                            VA_STATUS_ERROR_OPERATION_FAILED;
            }
            mtkvcp_log("stream start: VCP busy, retry %d", tries);
            pthread_mutex_unlock(&d->lock);
            usleep(250000);
            pthread_mutex_lock(&d->lock);
            if (!d->contexts[ci].in_use || c->vfd < 0) {
                mtkvcp_release_hw(d, ci);
                return VA_STATUS_ERROR_INVALID_CONTEXT;
            }
        }
    }
    c->streaming = 1;
    return VA_STATUS_SUCCESS;
}

static void mtkvcp_drop_pending(struct mtkvcp_context *c, int si);
static int mtkvcp_dec_pump(struct mtkvcp_drv *d, int ci, int block,
                           int timeout_ms);
static int mtkvcp_dec_recover(struct mtkvcp_drv *d, int ci);
static int mtkvcp_dec_flush_reset(struct mtkvcp_drv *d, int ci);
static int mtkvcp_wedge_recover(void);

VAStatus mtkvcp_dec_begin(struct mtkvcp_drv *d, int ci, int si)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    struct mtkvcp_surface *s = &d->surfaces[si];
    mtkvcp_log("begin ci=%d si=%d sctx=%d kind=%d state=%d %dx%d",
               ci, si, s->ctx, s->kind, s->state, s->width, s->height);
    if (c->error)
        return VA_STATUS_ERROR_DECODING_ERROR;
    if (c->au_target >= 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (s->ctx < 0) {
        /* First use binds the surface to this context. It holds no
         * buffer until a completion assigns one (cap_index -1).
         * The context follows the coded size while clients such as
         * VLC's GL converter size their pool to the visible frame
         * (e.g. 2460x1080 into 2496x1088), so accept any surface that
         * fits inside the decoded frame and crop to it on export. */
        if (s->width > c->width || s->height > c->height ||
            s->width < MTKVCP_MIN_W || s->height < MTKVCP_MIN_H)
            return VA_STATUS_ERROR_INVALID_SURFACE;
        mtkvcp_dec_grow_capture(d, ci);
        if (s->prime_fd >= 0) {
            close(s->prime_fd);
            s->prime_fd = -1;
        }
        s->ctx = ci;
        s->kind = MTKVCP_SURF_DECODE;
        s->cap_index = -1;
        s->state = MTKVCP_SS_IDLE;
    } else if (s->ctx != ci || s->kind != MTKVCP_SURF_DECODE) {
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    if (s->state == MTKVCP_SS_TARGET) {
        /* The surface still carries a submitted picture: the client reused
         * it without syncing (VLC drops late frames, and does so heavily
         * while the user drags the seek bar). Wait only briefly for a
         * picture that is about to finish; otherwise drop it and decode
         * into the surface again. This path must never block for seconds
         * and must never flush the tail: the client is actively decoding,
         * and a flush here collapses its Begin/End pairing.
         */
        int waited = 0;

        while (s->state == MTKVCP_SS_TARGET && waited < 100 &&
               !c->error) {
            if (mtkvcp_dec_pump(d, ci, 1, 20) < 0)
                break;
            waited += 20;
        }
        if (s->state == MTKVCP_SS_TARGET) {
            mtkvcp_drop_pending(c, si);
            mtkvcp_log("reuse: si=%d drops held picture", si);
            s->state = MTKVCP_SS_IDLE;
        }
    }
    if (s->state == MTKVCP_SS_READY) {
        /* Recycle: hand the finished buffer back to the driver. A frame
         * whose buffer was lost to a queue restart (CAPTURE kick) has no
         * buffer to return: drop it and decode into the surface again
         * rather than failing the client's pipeline. */
        if (!c->streaming || s->cap_index < 0) {
            mtkvcp_log("recycle si=%d: no buffer (streaming=%d cap=%d), "
                       "dropped", si, c->streaming, s->cap_index);
            s->state = MTKVCP_SS_IDLE;
        } else {
            mtkvcp_log("recycle si=%d buf=%d back to driver", si,
                       s->cap_index);
            if (mtkvcp_qbuf_cap(c, s->cap_index) < 0) {
                mtkvcp_log("recycle si=%d buf=%d qbuf failed", si,
                           s->cap_index);
                return VA_STATUS_ERROR_OPERATION_FAILED;
            }
            s->state = MTKVCP_SS_IDLE;
        }
    } else if (s->state != MTKVCP_SS_IDLE) {
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    c->au_target = si;
    c->au_len = 0;
    c->au_has_pic_param = 0;
    c->au_error = 0;
    c->custom_scaling = 0;
    c->h264_pic_valid = 0;
    c->h264_slice_seen = 0;
    c->hevc_pic_valid = 0;
    c->mp2_pic_valid = 0;
    c->mp2_iq_valid = 0;
    c->nslices = 0;
    return VA_STATUS_SUCCESS;
}

VAStatus mtkvcp_dec_render(struct mtkvcp_drv *d, int ci, int bi)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    struct mtkvcp_buffer *b = &d->buffers[bi];
    size_t add;
    if (c->au_target < 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    switch (b->type) {
    case VAPictureParameterBufferType:
        c->au_has_pic_param = 1;
        if (c->out_fourcc == V4L2_PIX_FMT_H264 &&
            (size_t)b->size * (size_t)b->num_elements >=
                sizeof(c->h264_pic)) {
            memcpy(&c->h264_pic, b->data, sizeof(c->h264_pic));
            c->h264_pic_valid = 1;
            {
                /* Debug: which surfaces does this picture reference?
                 * A recycled buffer must never be live here. */
                int q;
                char refs[128];
                size_t o = 0;
                for (q = 0; q < 16; q++) {
                    const VAPictureH264 *rp =
                        &c->h264_pic.ReferenceFrames[q];
                    if ((rp->flags & VA_PICTURE_H264_INVALID) ||
                        (int)rp->picture_id <= 0)
                        continue;
                    o += (size_t)snprintf(refs + o,
                                          o < sizeof(refs) ? sizeof(refs) - o : 0,
                                          "%ss%d", o ? "," : "",
                                          (int)rp->picture_id);
                    if (o >= sizeof(refs))
                        break;
                }
                refs[o < sizeof(refs) ? o : sizeof(refs) - 1] = 0;
                mtkvcp_log("h264 refs: %s",
                           o ? refs : "(none)");
            }
        } else if (c->out_fourcc == V4L2_PIX_FMT_HEVC &&
                   (size_t)b->size * (size_t)b->num_elements >=
                       sizeof(c->hevc_pic)) {
            memcpy(&c->hevc_pic, b->data, sizeof(c->hevc_pic));
            c->hevc_pic_valid = 1;
            /* num_short_term_ref_pic_sets is how many SPS-level RPS the
             * stream defines. st_rps_bits is the bit length of a
             * slice-embedded RPS, and is 0 when the slice references an
             * SPS one instead. mtkvcp_hevc_write_sps() declares zero SPS
             * RPS, so any stream with num_short_term_ref_pic_sets > 0 has
             * slices the firmware cannot resolve a reference list for. */
            mtkvcp_log("hevc pic: num_st_rps=%u st_rps_bits=%u "
                       "NoPicReordering=%u max_dpb_minus1=%u",
                       c->hevc_pic.num_short_term_ref_pic_sets,
                       c->hevc_pic.st_rps_bits,
                       c->hevc_pic.pic_fields.bits.NoPicReorderingFlag,
                       c->hevc_pic.sps_max_dec_pic_buffering_minus1);
        } else if (c->out_fourcc == V4L2_PIX_FMT_MPEG2 &&
                   (size_t)b->size * (size_t)b->num_elements >=
                       sizeof(c->mp2_pic)) {
            memcpy(&c->mp2_pic, b->data, sizeof(c->mp2_pic));
            c->mp2_pic_valid = 1;
        }
        break;
    case VASliceParameterBufferType:
        if (c->out_fourcc == V4L2_PIX_FMT_MPEG2 &&
            (size_t)b->size * (size_t)b->num_elements >=
                sizeof(VASliceParameterBufferMPEG2)) {
            VASliceParameterBufferMPEG2 *sp =
                (VASliceParameterBufferMPEG2 *)b->data;
            mtkvcp_log("mp2 slice: size=%u off=%u flag=0x%x mb_off=%u",
                       sp->slice_data_size, sp->slice_data_offset,
                       sp->slice_data_flag, sp->macroblock_offset);
        }
        if (c->out_fourcc == V4L2_PIX_FMT_H264 &&
            (size_t)b->size * (size_t)b->num_elements >=
                sizeof(VASliceParameterBufferH264) && !c->h264_slice_seen) {
            VASliceParameterBufferH264 *sp =
                (VASliceParameterBufferH264 *)b->data;
            c->h264_ref_l0 = sp->num_ref_idx_l0_active_minus1;
            c->h264_ref_l1 = sp->num_ref_idx_l1_active_minus1;
            c->h264_slice_seen = 1;
            mtkvcp_log("slice param: data_size=%u off=%u flag=0x%x "
                       "bit_off=%u ref_l0=%u ref_l1=%u",
                       sp->slice_data_size, sp->slice_data_offset,
                       sp->slice_data_flag, sp->slice_data_bit_offset,
                       c->h264_ref_l0, c->h264_ref_l1);
            /* slice_data_bit_offset points at the MB payload inside
             * the buffer (stateless hint); the NAL bytes are whole
             * (buffer size matches the Annex-B NAL size exactly). */
            (void)sp->slice_data_bit_offset;
        }
        break;
    case VAIQMatrixBufferType:
        if (c->out_fourcc == V4L2_PIX_FMT_MPEG2 &&
            (size_t)b->size * (size_t)b->num_elements >=
                sizeof(c->mp2_iq)) {
            memcpy(&c->mp2_iq, b->data, sizeof(c->mp2_iq));
            c->mp2_iq_valid = 1;
            mtkvcp_log("mp2 iq flags=%d/%d/%d/%d intra0-7=%02x%02x%02x%02x"
                       "%02x%02x%02x%02x nonintra0-7=%02x%02x%02x%02x"
                       "%02x%02x%02x%02x",
                       c->mp2_iq.load_intra_quantiser_matrix,
                       c->mp2_iq.load_non_intra_quantiser_matrix,
                       c->mp2_iq.load_chroma_intra_quantiser_matrix,
                       c->mp2_iq.load_chroma_non_intra_quantiser_matrix,
                       c->mp2_iq.intra_quantiser_matrix[0],
                       c->mp2_iq.intra_quantiser_matrix[1],
                       c->mp2_iq.intra_quantiser_matrix[2],
                       c->mp2_iq.intra_quantiser_matrix[3],
                       c->mp2_iq.intra_quantiser_matrix[4],
                       c->mp2_iq.intra_quantiser_matrix[5],
                       c->mp2_iq.intra_quantiser_matrix[6],
                       c->mp2_iq.intra_quantiser_matrix[7],
                       c->mp2_iq.non_intra_quantiser_matrix[0],
                       c->mp2_iq.non_intra_quantiser_matrix[1],
                       c->mp2_iq.non_intra_quantiser_matrix[2],
                       c->mp2_iq.non_intra_quantiser_matrix[3],
                       c->mp2_iq.non_intra_quantiser_matrix[4],
                       c->mp2_iq.non_intra_quantiser_matrix[5],
                       c->mp2_iq.non_intra_quantiser_matrix[6],
                       c->mp2_iq.non_intra_quantiser_matrix[7]);
        }
        if (c->out_fourcc == V4L2_PIX_FMT_HEVC &&
            c->hevc_pic_valid &&
            c->hevc_pic.pic_fields.bits.scaling_list_enabled_flag) {
            /* HEVC lists: 4x4/8x8/16x16/DC16/DC32; flat is 16,
             * absent is 0 (ffmpeg default-fill). */
            uint8_t *m = b->data;
            size_t n = (size_t)b->size * (size_t)b->num_elements, k;
            if (n > 990)
                n = 990;
            for (k = 0; k < n; k++)
                if (m[k] != 16 && m[k] != 0) {
                    mtkvcp_log("custom HEVC scaling entry -> refuse");
                    c->custom_scaling = 1;
                    break;
                }
        }
        if (c->out_fourcc == V4L2_PIX_FMT_H264) {
            /* Scan only the defined lists (6x16 + 2x64 = 224 bytes);
             * trailing reserved words are not scaling data.
             * Flat/default lists are all 16; ffmpeg zero-fills when
             * the stream carries none. Anything else is custom. */
            uint8_t *m = b->data;
            size_t n = (size_t)b->size * (size_t)b->num_elements, k;
            if (n > 224)
                n = 224;
            for (k = 0; k < n; k++)
                if (m[k] != 16 && m[k] != 0) {
                    mtkvcp_log("custom scaling list entry %u @%zu"
                               " -> refuse", m[k], k);
                    c->custom_scaling = 1;
                    break;
                }
        }
        break;
    case VASliceDataBufferType:
        add = (size_t)b->size * (size_t)b->num_elements;
        if (add == 0)
            break;
        /* Pure concatenation; EndPicture frames the AU using the
         * recorded slice boundaries (one NAL per slice buffer in the
         * ffmpeg convention; Annex-B clients keep their codes). */
        if (c->au_len + add > MTKVCP_OUT_SIZE_MAX)
            return VA_STATUS_ERROR_DECODING_ERROR;
        if (c->au_len + add > c->au_cap) {
            size_t ncap = c->au_cap ? c->au_cap * 2 : 65536;
            uint8_t *nd;
            while (ncap < c->au_len + add)
                ncap *= 2;
            if (ncap > MTKVCP_OUT_SIZE_MAX)
                ncap = MTKVCP_OUT_SIZE_MAX;
            nd = realloc(c->au, ncap);
            if (!nd)
                return VA_STATUS_ERROR_ALLOCATION_FAILED;
            c->au = nd;
            c->au_cap = ncap;
        }
        if (c->nslices < 40 &&
            (c->out_fourcc == V4L2_PIX_FMT_H264 ||
             c->out_fourcc == V4L2_PIX_FMT_HEVC))
            c->slice_offs[c->nslices++] = (int)c->au_len;
        memcpy(c->au + c->au_len, b->data, add);
        c->au_len += add;
        break;
    case VAHuffmanTableBufferType:
    case VAProbabilityBufferType:
        /* Stateful firmware parses; informational only. */
        break;
    default:
        break;
    }
    return VA_STATUS_SUCCESS;
}

/* Reap returned OUTPUT buffers (non-blocking). */
static void mtkvcp_reap_out(struct mtkvcp_context *c)
{
    for (;;) {
        struct v4l2_buffer b;
        struct v4l2_plane pl;
        memset(&b, 0, sizeof(b));
        memset(&pl, 0, sizeof(pl));
        b.type = DEC_OUT_TYPE;
        b.memory = V4L2_MEMORY_MMAP;
        b.length = 1;
        b.m.planes = &pl;
        if (mtkvcp_ioctl(c->vfd, VIDIOC_DQBUF, &b) < 0)
            return;
        if (b.index < (uint32_t)c->out_count)
            c->out_free[b.index] = 1;
    }
}

/* Tear down CAPTURE and rebuild it at the event geometry, keeping
 * OUTPUT streaming and all picture/surface state. Probe-identical
 * recipe: STREAMOFF CAP, REQBUFS 0, G_FMT, S_FMT, REQBUFS, QBUF all,
 * STREAMON CAP. Returns 0 ok, <0 fatal. */
static int mtkvcp_dec_renegotiate(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    struct v4l2_format g;
    int bound[MTKVCP_MAX_SURFACES];
    int nbound = 0, i, r, granted, ev_stride, ev_bh;
    mtkvcp_log("renegotiate: rebuilding CAPTURE at event geometry");
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
        if (d->surfaces[i].in_use && d->surfaces[i].ctx == ci &&
            d->surfaces[i].kind == MTKVCP_SURF_DECODE &&
            d->surfaces[i].state == MTKVCP_SS_READY) {
            /* Frames already delivered: a later event is a real
             * mid-stream change, not initial negotiation. */
            mtkvcp_log("renegotiate: delivered frames exist -> DRC");
            return -1;
        }
    /* Read the parsed geometry before touching the queue. If the buffers
     * allocated at stream start already match it -- the client sized the
     * context to the coded frame, which is the normal case -- restarting
     * CAPTURE is all the kernel needs (STREAMON clears wait_capture).
     * Reallocating a dozen multi-megabyte buffers here costs seconds and
     * starves the decode-ahead a B-pyramid needs to flush its reorder
     * window, which otherwise leaves the first sync waiting for 2.5s and
     * trips the tail flush. */
    memset(&g, 0, sizeof(g));
    g.type = DEC_CAP_TYPE;
    if (mtkvcp_ioctl(c->vfd, VIDIOC_G_FMT, &g) < 0) {
        mtkvcp_log("renegotiate: G_FMT failed");
        return -1;
    }
    ev_stride = (int)g.fmt.pix_mp.plane_fmt[0].bytesperline;
    ev_bh = (int)g.fmt.pix_mp.height;
    if (c->cap_count == c->pool_size &&
        c->cap_mmap_count == c->pool_size &&
        ev_stride == c->cap_stride && ev_bh == c->cap_bh) {
        mtkvcp_log("renegotiate: geometry unchanged (%dx%d stride=%d), "
                   "restart CAPTURE", ev_bh, ev_stride, ev_stride);
        mtkvcp_v4l2_stream(c->vfd, DEC_CAP_TYPE, 0);
        c->cap_queued = 0;
        for (i = 0; i < c->pool_size; i++) {
            if (mtkvcp_qbuf_cap(c, i) < 0) {
                mtkvcp_log("renegotiate: requeue buf=%d failed", i);
                return -1;
            }
        }
        if (mtkvcp_v4l2_stream(c->vfd, DEC_CAP_TYPE, 1) < 0) {
            mtkvcp_log("renegotiate: CAP STREAMON failed");
            return -1;
        }
        return 0;
    }
    mtkvcp_log("renegotiate: event geometry %ux%u",
               g.fmt.pix_mp.width, g.fmt.pix_mp.height);
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++) {
        if (c->cap_export[i]) {
            close(c->cap_export[i] - 1);
            c->cap_export[i] = 0;
        }
        if (c->cap_export_uv[i]) {
            close(c->cap_export_uv[i] - 1);
            c->cap_export_uv[i] = 0;
        }
    }
    mtkvcp_v4l2_stream(c->vfd, DEC_CAP_TYPE, 0);
    mtkvcp_v4l2_reqbufs(c->vfd, DEC_CAP_TYPE, 0);
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
        mtkvcp_unmap_cap(c, i);
    {
        struct v4l2_format f;
        memset(&f, 0, sizeof(f));
        f.type = DEC_CAP_TYPE;
        f.fmt.pix_mp.width = g.fmt.pix_mp.width;
        f.fmt.pix_mp.height = g.fmt.pix_mp.height;
        f.fmt.pix_mp.pixelformat = c->cap_fourcc;
        f.fmt.pix_mp.field = V4L2_FIELD_NONE;
        f.fmt.pix_mp.num_planes = (uint32_t)c->cap_planes;
        r = mtkvcp_ioctl(c->vfd, VIDIOC_S_FMT, &f);
        if (r < 0)
            mtkvcp_log("renegotiate: S_FMT errno=%d", r);
        else if (f.fmt.pix_mp.pixelformat != c->cap_fourcc)
            mtkvcp_log("renegotiate: fourcc changed 0x%x -> 0x%x",
                       c->cap_fourcc, f.fmt.pix_mp.pixelformat);
        if (r < 0 || f.fmt.pix_mp.pixelformat != c->cap_fourcc)
            return -1;
    }
    memset(&g, 0, sizeof(g));
    g.type = DEC_CAP_TYPE;
    if (mtkvcp_ioctl(c->vfd, VIDIOC_G_FMT, &g) == 0) {
        c->cap_stride = (int)g.fmt.pix_mp.plane_fmt[0].bytesperline;
        c->cap_bh = (int)g.fmt.pix_mp.height;
    }
    if (c->cap_stride <= 0)
        c->cap_stride = c->width;
    if (c->cap_bh < c->height)
        c->cap_bh = c->height;
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
        if (d->surfaces[i].in_use && d->surfaces[i].ctx == ci &&
            d->surfaces[i].kind == MTKVCP_SURF_DECODE &&
            d->surfaces[i].cap_index >= 0)
            bound[nbound++] = i;
    granted = mtkvcp_v4l2_reqbufs(c->vfd, DEC_CAP_TYPE, c->pool_size);
    if (granted < c->pool_size) {
        mtkvcp_log("renegotiate: REQBUFS granted %d < %d", granted,
                   c->pool_size);
        return -1;
    }
    c->cap_count = c->pool_size;
    c->cap_mmap_count = c->pool_size;
    for (i = 0; i < nbound && i < c->pool_size; i++) {
        d->surfaces[bound[i]].cap_index = i;
        if (d->surfaces[bound[i]].prime_fd >= 0) {
            close(d->surfaces[bound[i]].prime_fd);
            d->surfaces[bound[i]].prime_fd = -1;
        }
    }
    c->cap_queued = 0;
    for (i = 0; i < c->pool_size; i++) {
        if (mtkvcp_map_cap(c, i) < 0)
            return -1;
        /* States (IDLE/TARGET) are kept: in-flight pictures continue
         * against the new indices; unowned slots are spares. */
        if (mtkvcp_qbuf_cap(c, i) < 0)
            return -1;
    }
    mtkvcp_log("renegotiate: %d buffers sizeimage=%zu stride=%d bh=%d",
               nbound, c->cap_len[0], c->cap_stride, c->cap_bh);
    if (mtkvcp_v4l2_stream(c->vfd, DEC_CAP_TYPE, 1) < 0) {
        mtkvcp_log("renegotiate: CAP STREAMON failed");
        return -1;
    }
    return 0;
}

/* Drain pending DQEVENTs. The first SOURCE_CHANGE carries the parsed
 * stream geometry and triggers a CAPTURE rebuild (state-preserving).
 * Later size changes are mid-stream DRC, fatal in v1. */
static void mtkvcp_drain_events(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    for (;;) {
        struct v4l2_event ev;
        memset(&ev, 0, sizeof(ev));
        if (mtkvcp_ioctl(c->vfd, VIDIOC_DQEVENT, &ev) < 0)
            return;
        if (ev.type != V4L2_EVENT_SOURCE_CHANGE) {
            mtkvcp_log("event type=%u (ignored)", ev.type);
            continue;
        }
        if (!(ev.u.src_change.changes & V4L2_EVENT_SRC_CH_RESOLUTION)) {
            mtkvcp_log("non-resolution source change -> fatal");
            c->error = 1;
            return;
        }
        if (!c->negotiated) {
            /* First event: rebuild CAPTURE at parsed geometry. */
            if (mtkvcp_dec_renegotiate(d, ci) < 0) {
                c->error = 1;
                return;
            }
            c->negotiated = 1;
            continue;
        }
        {
            struct v4l2_format g;
            memset(&g, 0, sizeof(g));
            g.type = DEC_CAP_TYPE;
            if (mtkvcp_ioctl(c->vfd, VIDIOC_G_FMT, &g) < 0) {
                c->error = 1;
                return;
            }
            if ((int)g.fmt.pix_mp.width == c->width &&
                (int)g.fmt.pix_mp.height == c->cap_bh) {
                mtkvcp_log("repeat event, geometry unchanged");
                continue;
            }
        }
        mtkvcp_log("mid-stream DRC -> fatal (v1)");
        c->error = 1;
        return;
    }
}

/* Forget any in-flight picture targeting this surface. Used when a client
 * reuses a surface the firmware still holds: that frame is lost, but the
 * late CAPTURE completion recycles its buffer unmatched instead of marking
 * a surface that is already decoding again.
 */
static void mtkvcp_drop_pending(struct mtkvcp_context *c, int si)
{
    int i;

    for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
        if (c->pend[i].in_use && c->pend[i].surface == si)
            c->pend[i].in_use = 0;
}

/* Complete one CAPTURE buffer. Returns 1 if a picture completed. */
static int mtkvcp_complete_cap(struct mtkvcp_drv *d, int ci,
                               int idx, uint64_t seq, int last)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    int si_owner = -1, si_target = -1, i;
    int old_cap;
    (void)last;
    /* Oldest pending picture normally matches (FIFO firmware).
     * Match is by exact timestamp only: the driver passes OUTPUT
     * stamps through, and a zero-stamp filler must never steal a
     * picture slot. */
    if (seq != 0 && c->pend_head != c->pend_tail) {
        struct mtkvcp_pending_pic *p = &c->pend[c->pend_head];
        if (p->in_use && p->seq == seq) {
            si_target = p->surface;
            p->in_use = 0;
            c->pend_head = (c->pend_head + 1) % MTKVCP_MAX_SURFACES;
        }
    }
    if (seq != 0 && si_target < 0) {
        for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
            if (c->pend[i].in_use && c->pend[i].seq == seq) {
                si_target = c->pend[i].surface;
                c->pend[i].in_use = 0;
                break;
            }
    }
    if (si_target < 0)
        return 0;
    /* Who owns the filled buffer? A queued client surface, or a
     * driver-held spare pool slot (no surface). Either way the
     * target takes it; the displaced buffer stays queued. */
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
        if (d->surfaces[i].in_use && d->surfaces[i].ctx == ci &&
            d->surfaces[i].kind == MTKVCP_SURF_DECODE &&
            d->surfaces[i].cap_index == idx &&
            (d->surfaces[i].state == MTKVCP_SS_IDLE ||
             d->surfaces[i].state == MTKVCP_SS_TARGET)) {
            si_owner = i;
            break;
        }
    old_cap = d->surfaces[si_target].cap_index;
    /* Exports are cached by CAPTURE slot, so buffer swaps preserve identity. */
    d->surfaces[si_target].cap_index = idx;
    d->surfaces[si_target].state = MTKVCP_SS_READY;
    if (si_owner >= 0 && si_owner != si_target)
        d->surfaces[si_owner].cap_index = old_cap;
    /* old_cap (or a spare) stays queued under its holder. */
    c->completed_one = 1;
    /* NOTE: do not checksum cap_map[idx] here. The mapping is MMAP, not
     * dma-buf, so a CPU read at completion time sees whatever the cache
     * holds - measured as all-zero for a stream whose frames demonstrably
     * contain data. Any buffer-content probe has to go through the same
     * path the client uses. */
    mtkvcp_log("complete seq=%llu -> target s%d buf=%d",
               (unsigned long long)seq, si_target + 1, idx);
    return 1;
}

/*
 * Pump completions. block!=0 waits (poll) up to timeout_ms (<0 forever).
 * Returns 1 if any CAPTURE picture completed, 0 if idle/timeout, <0 on
 * fatal error.
 */
static int mtkvcp_dec_pump(struct mtkvcp_drv *d, int ci, int block,
                           int timeout_ms)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    int completed = 0, waited = 0;
    mtkvcp_drain_events(d, ci);
    mtkvcp_reap_out(c);
    for (;;) {
        struct v4l2_buffer b;
        struct v4l2_plane pl[2];
        int r;
        memset(&b, 0, sizeof(b));
        memset(pl, 0, sizeof(pl));
        b.type = DEC_CAP_TYPE;
        b.memory = V4L2_MEMORY_MMAP;
        b.length = (uint32_t)(c->cap_planes > 1 ? 2 : 1);
        b.m.planes = pl;
        if (!block) {
            r = mtkvcp_ioctl(c->vfd, VIDIOC_DQBUF, &b);
            if (r == -EAGAIN)
                break;
            if (r == -EPIPE) {
                mtkvcp_log("CAP DQBUF EPIPE -> fatal");
                c->error = 1;
                return -1;
            }
            if (r < 0 && r != -EAGAIN)
                mtkvcp_log("CAP DQBUF err %d", r);
            if (r < 0)
                break;
        } else {
            struct pollfd pfd;
            int pr;
            pfd.fd = c->vfd;
            pfd.events = POLLIN | POLLPRI;
            pfd.revents = 0;
            /* Wait without the driver lock: the client's display thread
             * needs it to sync and export finished frames, and a starved
             * display is what stops CAPTURE buffers from coming back. */
            pthread_mutex_unlock(&d->lock);
            pr = poll(&pfd, 1, timeout_ms < 0 ? -1 : timeout_ms);
            pthread_mutex_lock(&d->lock);
            r = pr;
            if (r <= 0) {
                if (completed)
                    break;
                return 0; /* timeout */
            }
            if (pfd.revents & POLLPRI) {
                mtkvcp_drain_events(d, ci);
                if (c->error)
                    return -1;
            }
            if (!(pfd.revents & POLLIN)) {
                if (completed)
                    break;
                if (timeout_ms >= 0 && (waited += timeout_ms) > 30000)
                    return 0;
                continue;
            }
            r = mtkvcp_ioctl(c->vfd, VIDIOC_DQBUF, &b);
            if (r == -EAGAIN)
                continue;
            if (r == -EPIPE) {
                c->error = 1;
                return -1;
            }
            if (r < 0)
                return -1;
        }
        /* The buffer left the kernel queue; keep the reserve count. */
        if (c->cap_queued > 0)
            c->cap_queued--;
        /* Zero-payload completions carry no picture (observed: stale
         * timestamps, no LAST). They must not consume a pending slot;
         * hand them straight back. */
        if (b.m.planes[0].bytesused == 0) {
            if (b.flags & V4L2_BUF_FLAG_LAST)
                c->saw_last = 1;
            mtkvcp_log("CAP DQBUF idx=%d empty filler flags=0x%x, requeue",
                       b.index, b.flags);
            mtkvcp_qbuf_cap(c, (int)b.index);
            mtkvcp_reap_out(c);
            if (!block)
                continue;
            break;
        }
        if (b.flags & V4L2_BUF_FLAG_LAST)
            c->saw_last = 1;
        mtkvcp_log("CAP DQBUF idx=%d ts=%llu bytes=%u flags=0x%x",
                   b.index,
                   (unsigned long long)mtkvcp_ts_to_seq(&b.timestamp),
                   b.m.planes[0].bytesused, b.flags);
        {
            int done = mtkvcp_complete_cap(d, ci, (int)b.index,
                                           mtkvcp_ts_to_seq(&b.timestamp),
                                           !!(b.flags & V4L2_BUF_FLAG_LAST));
            if (!done) {
                mtkvcp_log("CAP DQBUF idx=%d unattributed, requeue",
                           b.index);
                mtkvcp_qbuf_cap(c, (int)b.index);
            }
            completed |= done;
        }
        mtkvcp_reap_out(c);
        if (!block)
            continue;
        break;
    }
    return completed;
}

/* Framing sniff. Returns 0 Annex-B as-is, 1 length-prefixed (AVCC),
 * 2 Annex-B missing only the leading start code, <0 unframed. */
static int mtkvcp_sniff_framing(const uint8_t *p, size_t len)
{
    size_t i, zeros = 0;
    if (len >= 4 && !p[0] && !p[1] &&
        ((p[2] == 1) || (!p[2] && p[3] == 1)))
        return 0;
    /* Strict AVCC probe: 4-byte lengths must consume the AU exactly. */
    {
        size_t pos = 0;
        int nals = 0;
        while (pos + 4 <= len) {
            uint32_t n = ((uint32_t)p[pos] << 24) |
                         ((uint32_t)p[pos + 1] << 16) |
                         ((uint32_t)p[pos + 2] << 8) | p[pos + 3];
            if (!n || n > len - pos - 4)
                break;
            pos += 4 + n;
            nals++;
        }
        if (nals > 0 && pos == len)
            return 1;
    }
    /* Emulation prevention guarantees 00 00 01 only marks real NAL
     * boundaries, so a later start code means Annex-B with the first
     * code stripped (ffmpeg VAAPI convention). */
    for (i = 0; i + 2 < len; i++) {
        if (!p[i]) {
            zeros++;
            continue;
        }
        if (p[i] == 1 && zeros >= 2 && i > 2)
            return 2;
        zeros = 0;
    }
    (void)zeros;
    return -1;
}

/* Rewrite a length-prefixed AU to Annex-B in place (grows c->au).
 * Returns 0 ok, <0 if the framing is invalid. */
static int mtkvcp_avcc_to_annexb(struct mtkvcp_context *c)
{
    uint8_t *src = c->au;
    size_t slen = c->au_len, pos = 0, dlen = 0;
    uint8_t *dst;
    /* Worst case: every NAL is 1 byte -> 4x expansion. Cap sanely. */
    size_t cap = slen + slen / 16 + 4096;
    if (cap > MTKVCP_OUT_SIZE_MAX * 2)
        return -1;
    dst = malloc(cap ? cap : 1);
    if (!dst)
        return -1;
    while (pos + 4 <= slen) {
        uint32_t n = ((uint32_t)src[pos] << 24) |
                     ((uint32_t)src[pos + 1] << 16) |
                     ((uint32_t)src[pos + 2] << 8) | src[pos + 3];
        if (!n || n > slen - pos - 4)
            break;
        if (dlen + 4 + n > cap)
            break;
        dst[dlen++] = 0;
        dst[dlen++] = 0;
        dst[dlen++] = 0;
        dst[dlen++] = 1;
        memcpy(dst + dlen, src + pos + 4, n);
        dlen += n;
        pos += 4 + n;
    }
    if (pos != slen || !dlen) {
        free(dst);
        return -1;
    }
    free(c->au);
    c->au = dst;
    c->au_len = dlen;
    c->au_cap = cap;
    return 0;
}

static VAStatus mtkvcp_dec_submit_au(struct mtkvcp_drv *d, int ci);

/* Does this Annex-B AU carry an IDR slice? H.264 only; other codecs keep
 * the replay ring but never restart it from an IDR. */
static int mtkvcp_au_has_idr(const uint8_t *p, size_t len)
{
    size_t i;

    for (i = 0; i + 4 < len; i++) {
        size_t off;

        if (p[i] == 0 && p[i + 1] == 0 && p[i + 2] == 1)
            off = 3;
        else if (p[i] == 0 && p[i + 1] == 0 && p[i + 2] == 0 &&
                 p[i + 3] == 1)
            off = 4;
        else
            continue;
        if (i + off < len && (p[i + off] & 0x1f) == 5)
            return 1;
    }
    return 0;
}

/* Self-contained AU: H.264 IDR or HEVC IRAP (BLA/IDR/CRA). A resume that
 * starts here must not replay the pre-flush closure -- those access units
 * target surfaces the client may already be decoding into again, which
 * shows up either as old frames flashing past or as a stuck queue.
 */
static int mtkvcp_au_is_intra(const uint8_t *p, size_t len,
                              unsigned int fourcc)
{
    size_t i;

    for (i = 0; i + 4 < len; i++) {
        size_t off;
        unsigned int t;

        if (p[i] == 0 && p[i + 1] == 0 && p[i + 2] == 1)
            off = 3;
        else if (p[i] == 0 && p[i + 1] == 0 && p[i + 2] == 0 &&
                 p[i + 3] == 1)
            off = 4;
        else
            continue;
        if (i + off >= len)
            continue;
        if (fourcc == V4L2_PIX_FMT_H264) {
            if ((p[i + off] & 0x1f) == 5)
                return 1;
        } else if (fourcc == V4L2_PIX_FMT_HEVC) {
            t = (unsigned int)(p[i + off] >> 1) & 0x3f;
            if (t >= 16 && t <= 21)
                return 1;
        }
    }
    return 0;
}

/* Newest ring entry that decoded into surface si, or -1. */
static int mtkvcp_replay_find(struct mtkvcp_context *c, int si)
{
    int i;

    for (i = c->replay_count - 1; i >= 0; i--) {
        int idx = (c->replay_next - c->replay_count + i +
                   MTKVCP_REPLAY_MAX) % MTKVCP_REPLAY_MAX;

        if (c->replay_len[idx] && c->replay_target[idx] == si)
            return idx;
    }
    return -1;
}

/* Re-submit the pictures a mid-stream drain dropped. For H.264 that is the
 * transitive reference closure of the current picture, so only the live
 * references are re-decoded; other codecs (and any reference that has aged
 * out of the ring) fall back to the last-IDR window. Replay pictures carry
 * timestamp 0 and no pend record, so the driver treats their completions as
 * unattributed and requeues them. */
static void mtkvcp_dec_replay(struct mtkvcp_context *c, struct mtkvcp_drv *d,
                              int ci)
{
    unsigned char take[MTKVCP_REPLAY_MAX];
    int n = c->replay_count, i, q, fallback = 0;

    if (!n)
        return;
    memset(take, 0, sizeof(take));
    if ((c->out_fourcc == V4L2_PIX_FMT_H264 && c->h264_pic_valid) ||
        (c->out_fourcc == V4L2_PIX_FMT_HEVC && c->hevc_pic_valid)) {
        int work[MTKVCP_REPLAY_MAX];
        int seed[16], nseed = 0;
        int w = 0, k = 0;

        if (c->out_fourcc == V4L2_PIX_FMT_HEVC) {
            for (q = 0; q < 15; q++) {
                const VAPictureHEVC *rp =
                    &c->hevc_pic.ReferenceFrames[q];

                if ((rp->flags & VA_PICTURE_HEVC_INVALID) ||
                    (int)rp->picture_id <= 0)
                    continue;
                seed[nseed++] = (int)rp->picture_id - 1;
            }
        } else {
            for (q = 0; q < 16; q++) {
                const VAPictureH264 *rp =
                    &c->h264_pic.ReferenceFrames[q];

                if ((rp->flags & VA_PICTURE_H264_INVALID) ||
                    (int)rp->picture_id <= 0)
                    continue;
                seed[nseed++] = (int)rp->picture_id - 1;
            }
        }
        for (q = 0; q < nseed; q++) {
            int idx = mtkvcp_replay_find(c, seed[q]);

            if (idx < 0) {
                fallback = 1;
                break;
            }
            if (!take[idx]) {
                take[idx] = 1;
                work[w++] = idx;
            }
        }
        while (!fallback && k < w) {
            int idx = work[k++];

            for (q = 0; q < c->replay_nref[idx]; q++) {
                int ridx = mtkvcp_replay_find(c, c->replay_ref[idx][q]);

                if (ridx < 0) {
                    fallback = 1;
                    break;
                }
                if (!take[ridx]) {
                    take[ridx] = 1;
                    work[w++] = ridx;
                }
            }
        }
    } else {
        fallback = 1;
    }
    if (fallback) {
        int start = 0;

        memset(take, 0, sizeof(take));
        if (c->replay_idr >= 0) {
            int oldest = (c->replay_next - n + MTKVCP_REPLAY_MAX) %
                         MTKVCP_REPLAY_MAX;
            int dist = (c->replay_idr - oldest + MTKVCP_REPLAY_MAX) %
                       MTKVCP_REPLAY_MAX;

            if (dist < n)
                start = dist;
        }
        for (i = start; i < n; i++)
            take[(c->replay_next - n + i + MTKVCP_REPLAY_MAX) %
                 MTKVCP_REPLAY_MAX] = 1;
        mtkvcp_log("replay: fallback window %d AUs", n - start);
    }
    {
        int oldest = (c->replay_next - n + MTKVCP_REPLAY_MAX) %
                     MTKVCP_REPLAY_MAX;
        int submitted = 0;

        for (i = 0; i < n; i++) {
            int idx = (oldest + i) % MTKVCP_REPLAY_MAX;
            struct v4l2_buffer b;
            struct v4l2_plane pl;
            int slot = -1, k, waited = 0;

            if (!take[idx] || !c->replay_len[idx])
                continue;
            for (k = 0; k < c->out_count; k++)
                if (c->out_free[k]) {
                    slot = k;
                    break;
                }
            while (slot < 0 && waited < 10000 && !c->error) {
                if (mtkvcp_dec_pump(d, ci, 1, 1000) < 0)
                    break;
                for (k = 0; k < c->out_count; k++)
                    if (c->out_free[k]) {
                        slot = k;
                        break;
                    }
                waited += 1000;
            }
            if (slot < 0 || c->replay_len[idx] > c->out_len[slot])
                break;
            memcpy(c->out_map[slot], c->replay_au[idx], c->replay_len[idx]);
            memset(&b, 0, sizeof(b));
            memset(&pl, 0, sizeof(pl));
            b.type = DEC_OUT_TYPE;
            b.memory = V4L2_MEMORY_MMAP;
            b.index = (uint32_t)slot;
            b.length = 1;
            b.m.planes = &pl;
            b.m.planes[0].bytesused = (uint32_t)c->replay_len[idx];
            b.m.planes[0].length = (uint32_t)c->out_len[slot];
            b.timestamp.tv_sec = 0;
            b.timestamp.tv_usec = 0;
            if (mtkvcp_ioctl(c->vfd, VIDIOC_QBUF, &b) == 0) {
                c->out_free[slot] = 0;
                c->last_submit_ms = mtkvcp_now_ms();
                submitted++;
            }
        }
        mtkvcp_log("replay: closure resubmitted %d AUs", submitted);
    }
}

/* Build MPEG-2 PS with the given temporal ref and prepend to staged au. */
static VAStatus mtkvcp_mp2_prepend_ps(struct mtkvcp_drv *d, int ci,
                                      int temporal_ref)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    uint8_t ps[512];
    uint8_t *final;
    int pslen;
    const uint8_t *use_intra =
        (c->mp2_iq_valid && c->mp2_iq.load_intra_quantiser_matrix) ?
        c->mp2_iq.intra_quantiser_matrix : NULL;
    const uint8_t *use_non =
        (c->mp2_iq_valid &&
         c->mp2_iq.load_non_intra_quantiser_matrix) ?
        c->mp2_iq.non_intra_quantiser_matrix : NULL;
    mtkvcp_log("mpeg2 synth: type=%d fcode=0x%x temp=%d",
               c->mp2_pic.picture_coding_type, c->mp2_pic.f_code,
               temporal_ref);
    pslen = mtkvcp_mpeg2_build_ps(&c->mp2_pic,
                                  d->configs[c->config].profile,
                                  use_intra, use_non, temporal_ref, ps,
                                  sizeof(ps));
    if (pslen < 0 || c->au_len + (size_t)pslen > MTKVCP_OUT_SIZE_MAX) {
        mtkvcp_log("mpeg2 PS build failed -> fatal");
        return VA_STATUS_ERROR_DECODING_ERROR;
    }
    final = malloc(c->au_len + (size_t)pslen);
    if (!final)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    memcpy(final, ps, (size_t)pslen);
    memcpy(final + pslen, c->au, c->au_len);
    free(c->au);
    c->au = final;
    c->au_len += (size_t)pslen;
    c->au_cap = c->au_len;
    mtkvcp_log("prepended MPEG-2 PS (%d bytes)", pslen);
    return VA_STATUS_SUCCESS;
}

VAStatus mtkvcp_dec_end(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    VAStatus st;
    mtkvcp_log("EndPicture target=%d au_len=%zu has_pic=%d err=%d "
               "streaming=%d", c->au_target, c->au_len,
               c->au_has_pic_param, c->error, c->streaming);
    if (c->au_target < 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (c->au_len == 0) {
        c->au_target = -1;
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }
    if (c->error) {
        c->au_target = -1;
        c->au_len = 0;
        return VA_STATUS_ERROR_DECODING_ERROR;
    }
    /* Frame the AU for the Annex-B firmware interface:
     * - Annex-B in from the client passes through;
     * - whole-AU AVCC converts;
     * - raw slice NALs (ffmpeg: one NAL per slice buffer, SPS/PPS
     *   carried only in the picture params) gain per-NAL codes and
     *   synthesized parameter sets (H264; HEVC follows). */
    if (c->out_fourcc == V4L2_PIX_FMT_H264 ||
        c->out_fourcc == V4L2_PIX_FMT_HEVC) {
        int fr = c->au_len >= 4 ? mtkvcp_sniff_framing(c->au, c->au_len)
                                : -1;
        if (c->au_error)
            fr = -1;
        if (fr == 1) {
            if (mtkvcp_avcc_to_annexb(c) < 0)
                fr = -1;
            else
                mtkvcp_log("AVCC->AnnexB (%zu bytes)", c->au_len);
        } else if (fr == 2) {
            uint8_t *nd;
            if (c->au_len + 4 > MTKVCP_OUT_SIZE_MAX) {
                fr = -1;
            } else {
                nd = malloc(c->au_len + 4);
                if (!nd) {
                    fr = -1;
                } else {
                    nd[0] = nd[1] = nd[2] = 0;
                    nd[3] = 1;
                    memcpy(nd + 4, c->au, c->au_len);
                    free(c->au);
                    c->au = nd;
                    c->au_len += 4;
                    c->au_cap = c->au_len;
                    mtkvcp_log("prepended leading start code");
                }
            }
        }
        if (fr != 0 && fr != 1 &&
            c->out_fourcc == V4L2_PIX_FMT_HEVC) {
            /* Same raw-NAL collection as H264, with VPS/SPS/PPS. */
            uint8_t *framed;
            size_t fcap = c->au_len + (size_t)c->nslices * 6 + 2048;
            size_t flen = 0;
            int pps_ids[8], nids = 0, bad = 0;
            uint8_t ps[2048];
            int pslen = 0, k;
            if (c->au_error || c->custom_scaling || !c->hevc_pic_valid) {
                mtkvcp_log("raw HEVC picture without usable pic params "
                           "(err=%d scal=%d pic=%d) -> fatal",
                           c->au_error, c->custom_scaling,
                           c->hevc_pic_valid);
                c->au_target = -1;
                c->au_len = 0;
                return VA_STATUS_ERROR_DECODING_ERROR;
            }
            if (fcap > MTKVCP_OUT_SIZE_MAX * 2)
                bad = 1;
            framed = bad ? NULL : malloc(fcap ? fcap : 1);
            if (!framed)
                bad = 1;
            if (!bad) {
                for (k = 0; k < c->nslices; k++) {
                    size_t prev = (size_t)c->slice_offs[k];
                    size_t end = k + 1 < c->nslices ?
                                 (size_t)c->slice_offs[k + 1] : c->au_len;
                    size_t nlen;
                    int pid, has_code;
                    if (prev > end || end > c->au_len) {
                        bad = 1;
                        break;
                    }
                    nlen = end - prev;
                    has_code = nlen >= 4 && !c->au[prev] &&
                               !c->au[prev + 1] &&
                               ((c->au[prev + 2] == 1) ||
                                (!c->au[prev + 2] &&
                                 c->au[prev + 3] == 1));
                    if (flen + (has_code ? 0 : 4) + nlen > fcap) {
                        bad = 1;
                        break;
                    }
                    if (!has_code) {
                        framed[flen++] = 0;
                        framed[flen++] = 0;
                        framed[flen++] = 0;
                        framed[flen++] = 1;
                    }
                    pid = nlen > 2 ? mtkvcp_hevc_nal_pps_id(
                                         c->au + prev, nlen) : -1;
                    if (getenv("MTK_VCP_VA_TRACE_VERBOSE"))
                    mtkvcp_log("hevc slice %d off=%zu len=%zu pid=%d",
                               k, prev, nlen, pid);
                    if (pid < 0 || pid > 63) {
                        bad = 1;
                        break;
                    }
                    {
                        int q, have = 0;
                        for (q = 0; q < nids; q++)
                            if (pps_ids[q] == pid)
                                have = 1;
                        if (!have) {
                            if (nids >= 8) {
                                bad = 1;
                                break;
                            }
                            pps_ids[nids++] = pid;
                        }
                    }
                    memcpy(framed + flen, c->au + prev, nlen);
                    flen += nlen;
                }
            }
            if (!bad) {
                int prof = d->configs[c->config].profile ==
                           VAProfileHEVCMain10 ? 2 : 1;
                /* The kernel path sends the stream's own VPS/SPS/PPS, which
                 * appear once per sequence, not once per picture. Sending
                 * them with every access unit is a real structural
                 * difference; MTK_VCP_VA_PS_ONCE tests whether the firmware
                 * re-initialises its DPB each time it sees an SPS. */
                int send_ps = 1;

                if (mtkvcp_ps_once() && c->hevc_ps_sent &&
                    !memcmp(&c->hevc_ps_last, &c->hevc_pic,
                            sizeof(c->hevc_pic)))
                    send_ps = 0;
                if (send_ps) {
                    pslen = mtkvcp_hevc_build_ps(&c->hevc_pic, prof,
                                                 c->width, c->height,
                                                 pps_ids, nids, ps,
                                                 sizeof(ps));
                    mtkvcp_log("hevc build_ps -> %d (nids=%d)", pslen,
                               nids);
                    if (pslen < 0 || flen + (size_t)pslen > fcap)
                        bad = 1;
                    else {
                        memmove(framed + pslen, framed, flen);
                        memcpy(framed, ps, (size_t)pslen);
                        flen += (size_t)pslen;
                        memcpy(&c->hevc_ps_last, &c->hevc_pic,
                               sizeof(c->hevc_pic));
                        c->hevc_ps_sent = 1;
                    }
                }
            }
            if (bad) {
                free(framed);
                mtkvcp_log("HEVC parameter-set synthesis failed");
                c->au_target = -1;
                c->au_len = 0;
                return VA_STATUS_ERROR_DECODING_ERROR;
            }
            free(c->au);
            c->au = framed;
            c->au_len = flen;
            c->au_cap = fcap;
            mtkvcp_log("synthesized HEVC VPS/SPS/PPS (%d bytes)", pslen);
            fr = 0;
        }
        if (fr != 0 && fr != 1 &&
            c->out_fourcc == V4L2_PIX_FMT_H264) {
            /* Raw NALs at recorded slice boundaries: frame each,
             * then synthesize the missing parameter sets. */
            uint8_t *framed;
            size_t fcap = c->au_len + (size_t)c->nslices * 4 + 1024;
            size_t flen = 0;
            int pps_ids[8], nids = 0, bad = 0;
            uint8_t ps[1024];
            int pslen = 0;
            if (c->au_error || c->custom_scaling || !c->h264_pic_valid ||
                !c->h264_slice_seen) {
                mtkvcp_log("raw-NAL picture without usable pic params "
                           "(err=%d scal=%d pic=%d slice=%d) -> fatal",
                           c->au_error, c->custom_scaling,
                           c->h264_pic_valid, c->h264_slice_seen);
                c->au_target = -1;
                c->au_len = 0;
                return VA_STATUS_ERROR_DECODING_ERROR;
            }
            if (fcap > MTKVCP_OUT_SIZE_MAX * 2)
                bad = 1;
            framed = bad ? NULL : malloc(fcap ? fcap : 1);
            if (!framed)
                bad = 1;
            if (!bad) {
                int k;
                for (k = 0; k < c->nslices; k++) {
                    size_t prev = (size_t)c->slice_offs[k];
                    size_t end = k + 1 < c->nslices ?
                                 (size_t)c->slice_offs[k + 1] : c->au_len;
                    size_t nlen;
                    int pid, has_code;
                    if (prev > end || end > c->au_len) {
                        bad = 1;
                        break;
                    }
                    nlen = end - prev;
                    /* Pieces that already carry a code pass through;
                     * raw pieces gain one. */
                    has_code = nlen >= 4 && !c->au[prev] &&
                               !c->au[prev + 1] &&
                               ((c->au[prev + 2] == 1) ||
                                (!c->au[prev + 2] &&
                                 c->au[prev + 3] == 1));
                    if (flen + (has_code ? 0 : 4) + nlen > fcap) {
                        bad = 1;
                        break;
                    }
                    if (!has_code) {
                        framed[flen++] = 0;
                        framed[flen++] = 0;
                        framed[flen++] = 0;
                        framed[flen++] = 1;
                    }
                    /* pps_id from the slice header (1B NAL hdr). */
                    pid = nlen > 1 ? mtkvcp_h264_nal_pps_id(
                                         c->au + prev, nlen) : -1;
                    mtkvcp_log("slice %d off=%zu len=%zu pid=%d", k,
                               prev, nlen, pid);
                    if (pid < 0 || pid > 31) {
                        bad = 1;
                        break;
                    }
                    {
                        int q, have = 0;
                        for (q = 0; q < nids; q++)
                            if (pps_ids[q] == pid)
                                have = 1;
                        if (!have) {
                            if (nids >= 8) {
                                bad = 1;
                                break;
                            }
                            pps_ids[nids++] = pid;
                        }
                    }
                    memcpy(framed + flen, c->au + prev, nlen);
                    flen += nlen;
                    prev = end;
                }
            }
            mtkvcp_log("synth pre: bad=%d nslices=%d nids=%d", bad,
                       c->nslices, nids);
            if (!bad) {
                int cfg = d->configs[c->config].profile;
                mtkvcp_log("synth ids:%d ref=%u/%u", nids,
                           c->h264_ref_l0, c->h264_ref_l1);
                pslen = mtkvcp_h264_build_ps(&c->h264_pic, cfg, c->width,
                                             c->height, pps_ids, nids,
                                             c->h264_ref_l0,
                                             c->h264_ref_l1, ps,
                                             sizeof(ps));
                mtkvcp_log("build_ps -> %d", pslen);
                if (pslen < 0 || flen + (size_t)pslen > fcap)
                    bad = 1;
                else {
                    memmove(framed + pslen, framed, flen);
                    memcpy(framed, ps, (size_t)pslen);
                    flen += (size_t)pslen;
                }
            }
            if (bad) {
                free(framed);
                mtkvcp_log("parameter-set synthesis failed -> fatal");
                c->au_target = -1;
                c->au_len = 0;
                return VA_STATUS_ERROR_DECODING_ERROR;
            }
            free(c->au);
            c->au = framed;
            c->au_len = flen;
            c->au_cap = fcap;
            mtkvcp_log("synthesized SPS/PPS (%d bytes) + %d slices",
                       pslen, c->nslices);
            fr = 0;
        }
        if (fr < 0) {
            mtkvcp_log("unframed AU (%zu bytes) head=%02x%02x%02x%02x"
                       "%02x%02x%02x%02x -> fatal", c->au_len, c->au[0],
                       c->au[1], c->au[2], c->au[3], c->au[4], c->au[5],
                       c->au[6], c->au[7]);
            if (getenv("MTK_VCP_VA_DUMP_FAIL")) {
                FILE *fp = fopen("/tmp/va-au-fail.bin", "wb");
                if (fp) {
                    fwrite(c->au, 1, c->au_len, fp);
                    fclose(fp);
                }
            }
            c->au_target = -1;
            c->au_len = 0;
            return VA_STATUS_ERROR_DECODING_ERROR;
        }
        /* Annex-B with client-supplied parameter sets passes through
         * untouched (probe/harness path, byte-exact). */
    }
    /* MPEG-2: ffmpeg sends bare slices (sequence/picture headers
     * stripped). Rebuild them unless the client provided a sequence
     * header (probe bundles do). */
    if (c->out_fourcc == V4L2_PIX_FMT_MPEG2 && c->au_len >= 4) {
        size_t k;
        int has_seq = 0;
        for (k = 0; k + 3 < c->au_len; k++) {
            if (!c->au[k] && !c->au[k + 1] && c->au[k + 2] == 1 &&
                c->au[k + 3] == 0xb3) {
                has_seq = 1;
                break;
            }
            /* Bound the scan: headers live up front. */
            if (k > 4096 && has_seq == 0 && c->au[k] == 0 &&
                c->au[k + 1] == 0 && c->au[k + 2] == 1)
                break;
        }
        if (has_seq) {
            /* Client owns the headers (probe bundles); the firmware
             * caches them across AUs, so later bare slices resolve.
             * Remember this and never override with synthesis. */
            c->mp2_ps_seen = 1;
        }
        if (!has_seq && !c->mp2_ps_seen) {
            /* Display-order temporal refs without lookahead: N belief
             * (Bs per GOP, init IBBP), pending count, last anchor.
             * Absolute origin may shift on relock; gaps stay exact,
             * which is what prediction needs. */
            int is_b, ref;
            VAStatus st;
            if (!c->mp2_pic_valid) {
                mtkvcp_log("mpeg2 slices without pic params -> fatal");
                c->au_target = -1;
                c->au_len = 0;
                return VA_STATUS_ERROR_DECODING_ERROR;
            }
            is_b = c->mp2_pic.picture_coding_type == 3;
            if (c->mp2_last_anchor < 0) {
                /* First anchor (or pre-anchor Bs): clamp at 0. */
                ref = 0;
                if (!is_b)
                    c->mp2_last_anchor = 0;
            } else if (is_b) {
                ref = c->mp2_last_anchor - c->mp2_N + c->mp2_pending;
                if (ref < 0)
                    ref = 0;
                c->mp2_pending++;
            } else {
                /* The first inter-anchor interval carries no B-count
                 * information (Bs arrive after their anchor), so the
                 * N=2 belief stands exactly once; relock afterwards. */
                if (c->mp2_anchored_once &&
                    c->mp2_pending != c->mp2_N) {
                    mtkvcp_log("mpeg2 relock N %d -> %d", c->mp2_N,
                               c->mp2_pending);
                    c->mp2_N = c->mp2_pending;
                }
                c->mp2_anchored_once = 1;
                ref = c->mp2_last_anchor + c->mp2_N + 1;
                c->mp2_last_anchor = ref & 0x3ff;
                ref = c->mp2_last_anchor;
                c->mp2_pending = 0;
            }
            st = mtkvcp_mp2_prepend_ps(d, ci, ref);
            if (st != VA_STATUS_SUCCESS) {
                c->au_target = -1;
                c->au_len = 0;
                return st;
            }
        }
    }
    if (!c->streaming) {
        st = mtkvcp_dec_stream_start(d, ci);
        if (st != VA_STATUS_SUCCESS) {
            c->au_target = -1;
            c->au_len = 0;
            return st;
        }
    }
    if (c->stop_sent) {
        /* A tail flush was issued but the client kept submitting:
         * collect the drained tail, then resume. */
        struct v4l2_decoder_cmd cmd;
        int waited = 0, k, empty;
        for (;;) {
            empty = 1;
            for (k = 0; k < MTKVCP_MAX_SURFACES; k++)
                if (c->pend[k].in_use) {
                    empty = 0;
                    break;
                }
            if (c->saw_last && empty)
                break;
            if (waited > SYNC_TIMEOUT_MS ||
                mtkvcp_dec_pump(d, ci, 1, 500) < 0) {
                c->au_target = -1;
                c->au_len = 0;
                return VA_STATUS_ERROR_DECODING_ERROR;
            }
            waited += 500;
        }
        memset(&cmd, 0, sizeof(cmd));
        cmd.cmd = V4L2_DEC_CMD_START;
        if (mtkvcp_ioctl(c->vfd, VIDIOC_DECODER_CMD, &cmd) < 0) {
            mtkvcp_log("START after STOP failed");
            c->au_target = -1;
            c->au_len = 0;
            return VA_STATUS_ERROR_DECODING_ERROR;
        }
        c->stop_sent = 0;
        c->saw_last = 0;
        mtkvcp_log("resumed after tail flush");
        /* The drain's reset wiped the DPB. A dependent picture needs its
         * reference closure back; a self-contained IRAP/IDR (seek, channel
         * change) does not -- and replaying there would both show old
         * frames and target surfaces already in use again, so drop the
         * stale ring instead. */
        if (mtkvcp_au_is_intra(c->au, c->au_len, c->out_fourcc)) {
            mtkvcp_log("resume: intra AU, replay skipped");
            c->replay_count = 0;
            c->replay_idr = -1;
        } else {
            mtkvcp_dec_replay(c, d, ci);
        }
    }
    /* Hand the staged AU to the firmware (shared submit path, also
     * used for held B-frames whose headers are completed later). */
    return mtkvcp_dec_submit_au(d, ci);
}

/* Submit c->au for c->au_target: OUTPUT slot, QBUF, pend record. */
static VAStatus mtkvcp_dec_submit_au(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    struct v4l2_buffer b;
    struct v4l2_plane pl;
    int slot = -1, i, r;
    for (i = 0; i < c->out_count; i++)
        if (c->out_free[i]) {
            slot = i;
            break;
        }
    if (slot < 0) {
        /* A wedge is the firmware no longer retiring access units: the ring
         * never refills and the client blocks here. Waiting the full 15 s and
         * then failing the session is the worst of both outcomes - measured as
         * 15 s and 30 s stalls followed by a decoder that returns
         * VA_STATUS_ERROR_DECODING_ERROR for the rest of its life. Break the
         * ring instead, and keep the total budget short enough that a wedge
         * costs a hiccup rather than a session. */
        int waited = 0, tried = 0;

        while (slot < 0 && waited < 3000 && !c->error) {
            r = mtkvcp_dec_pump(d, ci, 1, 50);
            if (r < 0)
                break;
            for (i = 0; i < c->out_count; i++)
                if (c->out_free[i]) {
                    slot = i;
                    break;
                }
            waited += 50;
            if (slot < 0 && waited >= 400 && tried < 2 &&
                mtkvcp_wedge_recover()) {
                tried++;
                mtkvcp_log("submit: ring stalled %d ms, recovering (%d)",
                           waited, tried);
                if (mtkvcp_dec_recover(d, ci) < 0)
                    break;
            }
        }
        if (slot < 0) {
            mtkvcp_log("no OUTPUT slot freed after %dms -> fatal", waited);
            c->au_target = -1;
            c->au_len = 0;
            return VA_STATUS_ERROR_DECODING_ERROR;
        }
        if (tried)
            mtkvcp_log("submit: recovered in %d ms (%d attempt%s)",
                       waited, tried, tried == 1 ? "" : "s");
    }
    if (c->au_len > c->out_len[slot]) {
        c->au_target = -1;
        c->au_len = 0;
        return VA_STATUS_ERROR_DECODING_ERROR;
    }
    memcpy(c->out_map[slot], c->au, c->au_len);
    memset(&b, 0, sizeof(b));
    memset(&pl, 0, sizeof(pl));
    b.type = DEC_OUT_TYPE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = (uint32_t)slot;
    b.length = 1;
    b.m.planes = &pl;
    b.m.planes[0].bytesused = (uint32_t)c->au_len;
    b.m.planes[0].length = (uint32_t)c->out_len[slot];
    c->seq++;
    mtkvcp_seq_to_ts(c->seq, &b.timestamp);
    mtkvcp_log("OUT QBUF slot=%d len=%zu seq=%llu", slot, c->au_len,
               (unsigned long long)c->seq);
    {
        const char *ds = getenv("MTK_VCP_VA_DUMP_SEQ");
        if (ds && (uint64_t)atoll(ds) == c->seq + 1) {
            FILE *fp = fopen("/tmp/va-seq-dump.bin", "wb");
            if (fp) {
                fwrite(c->au, 1, c->au_len, fp);
                fclose(fp);
            }
        }
        /* Append every submitted access unit to one file. The units are
         * Annex-B with the synthesised parameter sets in front, so the
         * result is a decodable stream: feeding it back through another
         * decoder says whether this framing is what the firmware is
         * misreading, or whether the fault is elsewhere. Truncate the file
         * before the run; this only appends. */
        {
            const char *ap = getenv("MTK_VCP_VA_DUMP_AU");
            if (ap && *ap) {
                FILE *fp = fopen(ap, "ab");
                if (fp) {
                    fwrite(c->au, 1, c->au_len, fp);
                    fclose(fp);
                }
            }
        }
    }
    if (c->out_fourcc == V4L2_PIX_FMT_MPEG2)
        mtkvcp_log("mp2 submit: target=%d au_len=%zu nslices=%d temp=?",
                   c->au_target, c->au_len, c->nslices);
    r = mtkvcp_ioctl(c->vfd, VIDIOC_QBUF, &b);
    if (r < 0) {
        mtkvcp_log("OUT QBUF failed errno=%d (slot=%d known-free=%d)", r,
                   slot, c->out_free[slot]);
        if (getenv("MTK_VCP_VA_DUMP_FAIL")) {
            FILE *fp = fopen("/tmp/va-qbuf-fail.bin", "wb");
            if (fp) {
                fwrite(c->au, 1, c->au_len, fp);
                fclose(fp);
            }
        }
        c->au_target = -1;
        c->au_len = 0;
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }
    c->out_free[slot] = 0;
    c->last_submit_ms = mtkvcp_now_ms();
    {
        int tail = c->pend_tail;
        c->pend[tail].surface = c->au_target;
        c->pend[tail].seq = c->seq;
        c->pend[tail].in_use = 1;
        c->pend_tail = (tail + 1) % MTKVCP_MAX_SURFACES;
    }
    /* Buffer this AU so a mid-stream drain can rebuild the firmware DPB. */
    {
        int ring = c->replay_next;
        uint8_t *copy = malloc(c->au_len ? c->au_len : 1);

        if (copy) {
            memcpy(copy, c->au, c->au_len);
            free(c->replay_au[ring]);
            c->replay_au[ring] = copy;
            c->replay_len[ring] = c->au_len;
            if (c->replay_count < MTKVCP_REPLAY_MAX)
                c->replay_count++;
            if (c->out_fourcc == V4L2_PIX_FMT_H264 &&
                mtkvcp_au_has_idr(c->au, c->au_len))
                c->replay_idr = ring;
            /* Capture what this picture references so a late drain can
             * replay just the live reference closure. */
            c->replay_target[ring] = c->au_target;
            c->replay_nref[ring] = 0;
            if (c->out_fourcc == V4L2_PIX_FMT_H264 && c->h264_pic_valid) {
                int q;

                for (q = 0; q < 16; q++) {
                    const VAPictureH264 *rp =
                        &c->h264_pic.ReferenceFrames[q];

                    if ((rp->flags & VA_PICTURE_H264_INVALID) ||
                        (int)rp->picture_id <= 0)
                        continue;
                    if (c->replay_nref[ring] < 16)
                        c->replay_ref[ring][c->replay_nref[ring]++] =
                            (int)rp->picture_id - 1;
                }
            } else if (c->out_fourcc == V4L2_PIX_FMT_HEVC &&
                       c->hevc_pic_valid) {
                int q;

                for (q = 0; q < 15; q++) {
                    const VAPictureHEVC *rp =
                        &c->hevc_pic.ReferenceFrames[q];

                    if ((rp->flags & VA_PICTURE_HEVC_INVALID) ||
                        (int)rp->picture_id <= 0)
                        continue;
                    if (c->replay_nref[ring] < 16)
                        c->replay_ref[ring][c->replay_nref[ring]++] =
                            (int)rp->picture_id - 1;
                }
            }
            c->replay_next = (ring + 1) % MTKVCP_REPLAY_MAX;
        }
    }
    d->surfaces[c->au_target].state = MTKVCP_SS_TARGET;
    c->au_target = -1;
    c->au_len = 0;
    /* Opportunistic reap so SyncSurface usually finds work done. */
    mtkvcp_dec_pump(d, ci, 0, 0);
    return VA_STATUS_SUCCESS;
}

/* Send DEC_CMD_STOP once: flushes reorder-held tail frames. */
/* Restart the CAPTURE queue in place without reallocating buffers. The
 * kernel treats CAPTURE STREAMON as "the client restarted the queue", which
 * is what clears its draining/wait_capture state and lets queued OUTPUT
 * access units decode again after a flush-and-resume cycle.
 */
static int mtkvcp_dec_kick_capture(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    int i;

    /* The pool may have grown for a client that holds many surfaces, so
     * requeue everything that is mapped, not just the initial pool. */
    if (c->vfd < 0 || !c->streaming || c->cap_count <= 0 ||
        c->cap_mmap_count != c->cap_count) {
        mtkvcp_log("kick: not restartable ctx=%d streaming=%d mmap=%d/%d",
                   c->in_use, c->streaming, c->cap_mmap_count, c->cap_count);
        return -1;
    }
    mtkvcp_v4l2_stream(c->vfd, DEC_CAP_TYPE, 0);
    c->cap_queued = 0;
    /* STREAMOFF handed every buffer back to us. Frames that were in flight
     * are lost with the restart, so drop surface ownership before
     * requeueing: otherwise the client's next recycle would qbuf a buffer
     * that is already queued, and every following picture would fail. */
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
        if (d->surfaces[i].in_use && d->surfaces[i].ctx == ci) {
            d->surfaces[i].cap_index = -1;
            d->surfaces[i].state = MTKVCP_SS_IDLE;
            mtkvcp_drop_pending(c, i);
        }
    for (i = 0; i < c->cap_count; i++)
        if (mtkvcp_qbuf_cap(c, i) < 0) {
            mtkvcp_log("kick: requeue buf=%d failed", i);
            return -1;
        }
    if (mtkvcp_v4l2_stream(c->vfd, DEC_CAP_TYPE, 1) < 0)
        return -1;
    mtkvcp_log("kick: CAPTURE restarted in place");
    return 0;
}

static void mtkvcp_dec_send_stop(struct mtkvcp_context *c)
{
    c->resume_at_ms = 0;
    struct v4l2_decoder_cmd cmd;
    if (c->stop_sent || c->vfd < 0)
        return;
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd = V4L2_DEC_CMD_STOP;
    if (mtkvcp_ioctl(c->vfd, VIDIOC_DECODER_CMD, &cmd) == 0) {
        c->stop_sent = 1;
        mtkvcp_log("STOP sent (tail flush)");
    }
}

/* Whether the submit path may break a stalled OUTPUT ring itself. Set
 * MTK_VCP_VA_WEDGE_RECOVER=0 to get the old behaviour (wait 15 s, then fail
 * the session) for an A/B. */
static int mtkvcp_wedge_recover(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *v = getenv("MTK_VCP_VA_WEDGE_RECOVER");

        cached = (v && *v == '0') ? 0 : 1;
    }
    return cached;
}

/* Break a stalled OUTPUT ring with the vendor's flush.
 *
 * The kernel turns "STOP -> completed drain -> START" into
 * mtk_vcp_vdec_reset(decoder, false): AP_IPIMSG_DEC_RESET with drain_type 0,
 * i.e. the firmware drops its DPB and every access unit still in flight and
 * treats the next picture as a fresh sequence. That is the flush the vendor
 * driver issues when its OUTPUT queue is stopped (vdec_vcp_if.c: VDEC_FLUSH),
 * and the drain has to complete for it to happen at all.
 *
 * Two things have to be handed back for the drain to finish. The pending
 * records go first: a completion that matches no pending record is
 * unattributed, and the pump already returns those straight to the driver.
 * Then the CAPTURE buffers the client is holding - the client is blocked in
 * EndPicture while this runs and cannot recycle anything, and with the pool
 * full the firmware has nowhere to put the reorder tail, so it never reports
 * LAST (measured dstq=0 during the drain). These are frames a stalled session
 * is abandoning anyway.
 *
 * Called with d->lock held, from the submit path.
 */
static int mtkvcp_dec_flush_reset(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    struct v4l2_decoder_cmd cmd;
    int i, waited = 0, ok, requeued = 0;

    for (i = 0; i < MTKVCP_MAX_SURFACES; i++) {
        struct mtkvcp_surface *s = &d->surfaces[i];

        c->pend[i].in_use = 0;
        if (!s->in_use || s->ctx != ci)
            continue;
        if (s->cap_index >= 0) {
            mtkvcp_qbuf_cap(c, s->cap_index);
            s->cap_index = -1;
            requeued++;
        }
        if (s->state != MTKVCP_SS_IDLE)
            s->state = MTKVCP_SS_IDLE;
    }
    mtkvcp_log("flush: pool=%d requeued=%d queued=%d", c->cap_count,
               requeued, c->cap_queued);
    mtkvcp_dec_send_stop(c);
    for (;;) {
        int k, empty = 1;

        for (k = 0; k < MTKVCP_MAX_SURFACES; k++)
            if (c->pend[k].in_use) {
                empty = 0;
                break;
            }
        if ((c->saw_last && empty) || waited >= 300 || c->error)
            break;
        if (mtkvcp_dec_pump(d, ci, 1, 50) < 0)
            break;
        waited += 50;
    }
    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd = V4L2_DEC_CMD_START;
    ok = mtkvcp_ioctl(c->vfd, VIDIOC_DECODER_CMD, &cmd) == 0;
    if (!ok) {
        /* Draining but not stopped: the kernel refuses START until the
         * queue is restarted. */
        if (mtkvcp_dec_kick_capture(d, ci) == 0) {
            memset(&cmd, 0, sizeof(cmd));
            cmd.cmd = V4L2_DEC_CMD_START;
            ok = mtkvcp_ioctl(c->vfd, VIDIOC_DECODER_CMD, &cmd) == 0;
        }
    }
    if (!ok) {
        mtkvcp_log("flush: START refused after %d ms", waited);
        return -1;
    }
    mtkvcp_log("flush: drain %s after %d ms",
               c->saw_last ? "complete" : "abandoned", waited);
    c->stop_sent = 0;
    c->saw_last = 0;
    c->resume_at_ms = 0;
    c->hevc_ps_sent = 0;      /* the firmware dropped its parameter sets */
    c->replay_count = 0;
    c->replay_idr = -1;
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
        c->pend[i].in_use = 0;
    return 0;
}

/* Break a stalled OUTPUT ring: the firmware has stopped retiring access units,
 * so the ring never refills and the client blocks in Begin/EndPicture until a
 * watchdog gives up (measured: 15 s and 30 s stalls, then a session that keeps
 * returning VA_STATUS_ERROR_DECODING_ERROR for the rest of its life). This is
 * the same reset as the proactive one, forced through a CAPTURE restart.
 */
static int mtkvcp_dec_recover(struct mtkvcp_drv *d, int ci)
{
    int r = mtkvcp_dec_flush_reset(d, ci);

    mtkvcp_log("recover: ring broken, queue restarted (%d)", r);
    return r;
}

/* True if si's picture is the newest still pending. */
static int mtkvcp_is_tail(struct mtkvcp_drv *d, int ci, int si)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    uint64_t myseq = 0;
    int i, found = 0;
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
        if (c->pend[i].in_use && c->pend[i].surface == si) {
            myseq = c->pend[i].seq;
            found = 1;
            break;
        }
    if (!found)
        return 0;
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
        if (c->pend[i].in_use && c->pend[i].seq > myseq)
            return 0;
    return 1;
}

VAStatus mtkvcp_dec_sync(struct mtkvcp_drv *d, int si, uint64_t timeout_ns)
{
    struct mtkvcp_surface *s = &d->surfaces[si];
    int ci = s->ctx, waited = 0, slice = 250;
    int budget_ms = timeout_ns ? (int)(timeout_ns / 1000000ull) : -1;
    /* Which surface does the client actually wait on, and which CAPTURE
     * buffer does it hold? A client that syncs one surface repeatedly would
     * read a single picture even though the decoder produced distinct ones. */
    mtkvcp_log("sync si=%d state=%d cap_index=%d", si, s->state,
               s->cap_index);
    if (s->state == MTKVCP_SS_READY)
        return VA_STATUS_SUCCESS;
    if (s->state == MTKVCP_SS_ERROR)
        return VA_STATUS_ERROR_DECODING_ERROR;
    /* No work queued for this surface: VAAPI treats that as complete. It
     * happens after a CAPTURE restart dropped the frames in flight. */
    if (s->state == MTKVCP_SS_IDLE)
        return VA_STATUS_SUCCESS;
    if (s->state != MTKVCP_SS_TARGET)
        return VA_STATUS_ERROR_DECODING_ERROR; /* nothing in flight */
    if (ci < 0 || ci >= MTKVCP_MAX_CONTEXTS ||
        !d->contexts[ci].in_use)
        return VA_STATUS_ERROR_INVALID_SURFACE;
    if (d->contexts[ci].error)
        return VA_STATUS_ERROR_DECODING_ERROR;
    for (;;) {
        int r = mtkvcp_dec_pump(d, ci, 1, slice);
        if (r < 0 || d->contexts[ci].error)
            return VA_STATUS_ERROR_DECODING_ERROR;
        if (s->state == MTKVCP_SS_READY)
            return VA_STATUS_SUCCESS;
        waited += slice;
        /* Tail picture stalled: the client has stopped submitting
         * (end of stream) and the firmware holds reorder frames.
         * STOP flushes them; a later EndPicture sends START.
         * Gated on warm firmware: a cold VCP legitimately stalls
         * seconds on first frames (B-pyramid holds output until the
         * DPB fills), which must never look like end-of-stream.
         * Mid-stream backpressure self-protects: any newer submit
         * clears is_tail, and active submitters keep last_submit
         * fresh, so these can only fire when the client is truly
         * gone (or pathologically stalled for seconds). */
        if (!d->contexts[ci].stop_sent &&
            d->contexts[ci].completed_one && waited >= 2500 &&
            mtkvcp_now_ms() - d->contexts[ci].last_submit_ms > 2000 &&
            mtkvcp_is_tail(d, ci, si))
            mtkvcp_dec_send_stop(&d->contexts[ci]);
        /* Ordered clients stall on an earlier held frame while the
         * tail sits behind it. B-pyramid firmware holds output until
         * the DPB fills, so the very first picture can be held before
         * any completion has ever been attributed: gate on the client
         * going idle, not on completed_one, or a client that blocks on
         * the first sync never reaches the 30 s SYNC_TIMEOUT_MS with
         * any way to flush it. */
        if (!d->contexts[ci].stop_sent &&
            waited >= 3000 &&
            mtkvcp_now_ms() - d->contexts[ci].last_submit_ms > 3000)
            mtkvcp_dec_send_stop(&d->contexts[ci]);
        /* The flush released the reorder tail, but the kernel stays drained
         * until START; a client blocked on those frames (seek, end of
         * stream) never submits the access unit that used to send it, so
         * queued AUs would sit forever. Resume by ourselves shortly after
         * the flush, bounded so a pathological stream cannot loop. */
        if (d->contexts[ci].stop_sent) {
            uint64_t now = mtkvcp_now_ms();

            if (!d->contexts[ci].resume_at_ms)
                d->contexts[ci].resume_at_ms = now + 400;
            else if (now >= d->contexts[ci].resume_at_ms &&
                     d->contexts[ci].resume_count < 8) {
                struct v4l2_decoder_cmd cmd;

                int ok;

                memset(&cmd, 0, sizeof(cmd));
                cmd.cmd = V4L2_DEC_CMD_START;
                ok = mtkvcp_ioctl(d->contexts[ci].vfd,
                                  VIDIOC_DECODER_CMD, &cmd) == 0;
                if (!ok) {
                    /* Draining but not stopped: the kernel refuses START
                     * until the queue is restarted. */
                    if (mtkvcp_dec_kick_capture(d, ci) == 0) {
                        memset(&cmd, 0, sizeof(cmd));
                        cmd.cmd = V4L2_DEC_CMD_START;
                        ok = mtkvcp_ioctl(d->contexts[ci].vfd,
                                          VIDIOC_DECODER_CMD, &cmd) == 0;
                    }
                }
                if (ok) {
                    d->contexts[ci].stop_sent = 0;
                    d->contexts[ci].saw_last = 0;
                    d->contexts[ci].resume_at_ms = 0;
                    d->contexts[ci].resume_count++;
                    mtkvcp_log("resumed after drain (auto %d)",
                               d->contexts[ci].resume_count);
                } else if (!d->contexts[ci].resume_logged) {
                    d->contexts[ci].resume_logged = 1;
                    mtkvcp_log("auto-resume: START refused, kick failed");
                }
            }
        }
        if (budget_ms >= 0) {
            if (waited >= budget_ms)
                return VA_STATUS_ERROR_DECODING_ERROR;
        } else if (waited > SYNC_TIMEOUT_MS) {
            return VA_STATUS_ERROR_DECODING_ERROR;
        }
    }
}

/* Strict VAAPI semantics: returns only when the frame is ready. */
/* Stage decoded frames the client has not read yet into CPU memory.
 *
 * VAAPI keeps a surface valid after its context is destroyed, so the
 * frames still sitting in CAPTURE buffers must outlive the V4L2 session.
 * ffmpeg tears the decoder context down while its filter thread is still
 * reading the tail of the stream, so without this the final frame of
 * every stream is lost. The copy is only made for frames that completed
 * and were never handed to the client; anything already read keeps its
 * CAPTURE mapping until the context is gone.
 */
static void mtkvcp_dec_stage_unread(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    int i;
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++) {
        struct mtkvcp_surface *s = &d->surfaces[i];
        uint8_t *src;
        size_t y, need;
        if (!s->in_use || s->ctx != ci ||
            s->kind != MTKVCP_SURF_DECODE)
            continue;
        if (s->state != MTKVCP_SS_READY || s->cap_index < 0 ||
            s->cap_index >= c->cap_mmap_count || !c->cap_map[s->cap_index])
            continue;
        /* Layout matches the CPU-staged reader in GetImage: luma rows at
         * the V4L2 stride, chroma at enc_stride * visible height. */
        need = (size_t)c->cap_stride * (size_t)s->height * 3u / 2u;
        if (!need)
            continue;
        s->enc_data = malloc(need);
        if (!s->enc_data)
            continue;
        src = c->cap_map[s->cap_index];
        if (c->cap_tiled) {
            /* Plane 0 is only the luma tiles and the chroma tiles are a
             * second mapping, so the row copies below would walk straight
             * off the end of plane 0. Expand both planes instead, into the
             * layout the staged reader above expects.
             */
            const uint8_t *uv = c->cap_map_uv[s->cap_index];

            if (!uv) {
                free(s->enc_data);
                s->enc_data = NULL;
                continue;
            }
            mtkvcp_detile_mm21(s->enc_data, c->cap_stride, src,
                               c->cap_stride, s->height, 32);
            mtkvcp_detile_mm21(s->enc_data +
                                   (size_t)c->cap_stride * s->height,
                               c->cap_stride, uv, c->cap_stride,
                               s->height / 2, 16);
        } else {
            for (y = 0; y < (size_t)s->height; y++)
                memcpy(s->enc_data + y * (size_t)c->cap_stride,
                       src + y * (size_t)c->cap_stride,
                       (size_t)c->cap_stride);
            for (y = 0; y < (size_t)s->height / 2; y++)
                memcpy(s->enc_data +
                           ((size_t)s->height + y) * (size_t)c->cap_stride,
                       src + ((size_t)c->cap_bh + y) * (size_t)c->cap_stride,
                       (size_t)c->cap_stride);
        }
        s->enc_size = need;
        s->enc_stride = c->cap_stride;
        s->kind = MTKVCP_SURF_DECODE_STAGED;
        s->cap_index = -1;
    }
}

void mtkvcp_dec_destroy(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    int i, waited = 0;
    if (c->vfd < 0)
        return;
    {
        int k, has_pending = 0;
        for (k = 0; k < MTKVCP_MAX_SURFACES; k++)
            if (c->pend[k].in_use) {
                has_pending = 1;
                break;
            }
        /* Nothing in flight: no EOS round-trip needed. */
        if (!has_pending)
            c->saw_last = 1;
    }
    if (c->streaming && !c->error && !c->saw_last) {
        /* Drain: ask for EOS, then collect until LAST + empty. */
        struct v4l2_decoder_cmd cmd;
        memset(&cmd, 0, sizeof(cmd));
        cmd.cmd = V4L2_DEC_CMD_STOP;
        mtkvcp_ioctl(c->vfd, VIDIOC_DECODER_CMD, &cmd);
        while (waited < SYNC_TIMEOUT_MS) {
            int empty = (c->pend_head == c->pend_tail);
            int k, allempty = empty;
            if (allempty) {
                for (k = 0; k < MTKVCP_MAX_SURFACES; k++)
                    if (c->pend[k].in_use) {
                        allempty = 0;
                        break;
                    }
            }
            if (c->saw_last && allempty)
                break;
            if (mtkvcp_dec_pump(d, ci, 1, 500) < 0)
                break;
            waited += 500;
        }
    }
    if (c->streaming) {
        mtkvcp_v4l2_stream(c->vfd, DEC_OUT_TYPE, 0);
        mtkvcp_v4l2_stream(c->vfd, DEC_CAP_TYPE, 0);
        mtkvcp_v4l2_reqbufs(c->vfd, DEC_OUT_TYPE, 0);
        mtkvcp_v4l2_reqbufs(c->vfd, DEC_CAP_TYPE, 0);
        c->streaming = 0;
    } else {
        mtkvcp_v4l2_reqbufs(c->vfd, DEC_OUT_TYPE, 0);
    }
    /* VAAPI keeps a surface valid after its context is destroyed, so any
     * decoded frame the client has not read yet is copied out before the
     * CAPTURE mappings go away. ffmpeg tears the decoder context down
     * while its filter thread is still reading the tail of the stream;
     * without this the last frame of every stream is lost. */
    mtkvcp_dec_stage_unread(d, ci);
    mtkvcp_unmap_all(c);
    close(c->vfd);
    c->vfd = -1;
    mtkvcp_release_hw(d, ci);
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++) {
        c->pend[i].in_use = 0;
        if (d->surfaces[i].in_use && d->surfaces[i].ctx == ci) {
            d->surfaces[i].state = MTKVCP_SS_IDLE;
            d->surfaces[i].cap_index = -1;
        }
    }
}
