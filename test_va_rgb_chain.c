/* SPDX-License-Identifier: MIT */
/*
 * Packed-RGB capture chain harness.
 *
 * Reproduces the krdp/KPipeWire call sequence exactly:
 *
 *   import a compositor dma-buf as a VA surface  (hwmap=mode=direct)
 *   scale_vaapi=<out format>                     (VideoProc)
 *   encode the result as H.264                   (h264_vaapi)
 *
 * With MTK_VCP_VA_RGB_INPUT set, the post-processing step must carry the
 * RGB pixels through unchanged and the encoder must submit them to the
 * firmware as packed RGB, which converts them. The coded stream is then
 * decoded in software and compared against the pattern that went in, so a
 * wrong channel order or a stale CPU conversion shows up as a pixel
 * mismatch rather than as a passing run.
 *
 * Without the variable the same chain must still convert to NV12 on the
 * CPU and encode, which is the regression this harness also guards.
 *
 * Usage: test_va_rgb_chain [--w W] [--h H] [--frames N] <out.h264>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <sys/ioctl.h>
#include <time.h>
#include <linux/dma-heap.h>
#include <linux/dma-buf.h>
#include <libdrm/drm_fourcc.h>

#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_vpp.h>
#include <va/va_drmcommon.h>
#include <va/va_enc_h264.h>

/* How many frames the harness may keep in flight at once (--depth). The
 * backend pipelines that many OUTPUT slots, so this is what turns the
 * measurement from one-frame latency into sustained throughput.
 */
#define MTKVCP_HARN_DEPTH 8

static uint64_t harn_now_us(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

/* Wait for one submitted picture and account its coded bytes. */
static int harn_collect(VADisplay dpy, VABufferID coded, FILE *ofp,
                        int nodump, size_t *total)
{
    VACodedBufferSegment *seg;
    VAStatus st = vaSyncBuffer(dpy, coded, 0);

    if (st != VA_STATUS_SUCCESS) {
        fprintf(stderr, "FAIL: vaSyncBuffer -> %d\n", st);
        return -1;
    }
    st = vaMapBuffer(dpy, coded, (void **)&seg);
    if (st != VA_STATUS_SUCCESS) {
        fprintf(stderr, "FAIL: map coded -> %d\n", st);
        return -1;
    }
    if (!seg || !seg->buf || !seg->size) {
        fprintf(stderr, "FAIL: empty coded segment\n");
        vaUnmapBuffer(dpy, coded);
        return -1;
    }
    if (!nodump)
        fwrite(seg->buf, 1, seg->size, ofp);
    *total += seg->size;
    vaUnmapBuffer(dpy, coded);
    return 0;
}

/* Same test the backend uses: only a real dma-buf implements DMA_BUF_IOCTL_SYNC. */
static int fd_is_dmabuf(int fd)
{
    struct dma_buf_sync sync = { .flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW };
    struct dma_buf_sync end = { .flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW };
    int r;

    if (fd < 0)
        return 0;
    r = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
    if (r == 0)
        ioctl(fd, DMA_BUF_IOCTL_SYNC, &end);
    return r == 0;
}

#define DIE(...) do { fprintf(stderr, "FAIL: " __VA_ARGS__); \
                      fprintf(stderr, "\n"); return 1; } while (0)
#define CHECKST(fn, st) do { if ((st) != VA_STATUS_SUCCESS) { \
    fprintf(stderr, "FAIL: %s -> %d\n", fn, st); return 1; } } while (0)

static int memfd_new(const char *name, size_t size)
{
    int fd = (int)syscall(SYS_memfd_create, name, 0);
    if (fd < 0)
        return -1;
    if (ftruncate(fd, (off_t)size) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

/*
 * A real dma-buf from the system heap. The zero-copy hand-off needs a
 * genuine exporter behind the fd because the encoder's OUTPUT queue runs
 * in V4L2 DMABUF memory; a memfd can be mapped and copied from but cannot
 * be DMA'd from. Falls back to a memfd so the copy path stays testable.
 */
static int dmabuf_new(size_t size)
{
    struct dma_heap_allocation_data d = {
        .len = size,
        .fd_flags = O_RDWR | O_CLOEXEC,
    };
    int h = open("/dev/dma_heap/default_cma_region", O_RDWR | O_CLOEXEC);
    if (h < 0)
        h = open("/dev/dma_heap/system", O_RDWR | O_CLOEXEC);
    if (h < 0)
        return memfd_new("krdp-frame", size);
    if (ioctl(h, DMA_HEAP_IOCTL_ALLOC, &d) < 0) {
        close(h);
        return memfd_new("krdp-frame", size);
    }
    close(h);
    return (int)d.fd;
}

/*
 * The pattern the compositor would hand over: three saturated blocks and a
 * horizontal ramp, written as B,G,R,X - the byte order of
 * DRM_FORMAT_ARGB8888, which is what KPipeWire's dma-buf carries.
 */
static void fill_bgrx(uint8_t *m, int w, int h, int stride, int frame)
{
    int x, y;
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            uint8_t *p = m + (size_t)y * (size_t)stride + (size_t)x * 4u;
            int r, g, b;
            int v = (x * 255) / (w > 1 ? w - 1 : 1);
            if (y < h / 4 && x < w / 4) {
                r = 255; g = 0; b = 0;
            } else if (y < h / 4 && x < w / 2) {
                r = 0; g = 255; b = 0;
            } else if (y < h / 4 && x < 3 * w / 4) {
                r = 0; g = 0; b = 255;
            } else {
                /* Offset the ramp per frame so a stale frame is visible. */
                r = g = b = (v + frame * 8) & 0xff;
            }
            p[0] = (uint8_t)b;
            p[1] = (uint8_t)g;
            p[2] = (uint8_t)r;
            p[3] = 0xff;
        }
    }
}

/*
 * High-frequency noise, to stand in for real video content.
 *
 * The synthetic pattern above is mostly flat blocks plus a slow ramp, which
 * is close to free for an encoder: large uniform areas and a predictable
 * gradient compress to very few bits. Real screen content (a full-screen
 * video, a terminal, a browser) is full of edges and changes every frame.
 * Timing the chain on the smooth pattern measures the pipeline, not the
 * encoder's actual work, so this fills the buffer with per-pixel noise that
 * changes every frame.
 */
static void fill_noise(uint8_t *m, int w, int h, int stride, int frame)
{
    uint32_t s = 0x9e3779b9u ^ (uint32_t)frame * 2654435761u;
    int x, y;

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            uint8_t *p = m + (size_t)y * (size_t)stride + (size_t)x * 4u;

            /* xorshift keeps the values well spread without a table. */
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            p[0] = (uint8_t)s;
            p[1] = (uint8_t)(s >> 8);
            p[2] = (uint8_t)(s >> 16);
            p[3] = 0xff;
        }
    }
}

static void run_vpp(VADisplay dpy, VAContextID ctx, VASurfaceID in,
                    VASurfaceID target, int *status)
{
    VAProcPipelineParameterBuffer params;
    VABufferID buf;
    VAStatus st;
    memset(&params, 0, sizeof(params));
    params.surface = in;
    params.surface_region = NULL;
    params.output_region = NULL;
    params.surface_color_standard = VAProcColorStandardNone;
    params.output_color_standard = VAProcColorStandardBT601;
    params.filter_flags = VA_FILTER_SCALING_FAST;
    st = vaBeginPicture(dpy, ctx, target);
    if (st != VA_STATUS_SUCCESS) {
        fprintf(stderr, "FAIL: vpp BeginPicture -> %d\n", st);
        *status = 1;
        return;
    }
    st = vaCreateBuffer(dpy, ctx, VAProcPipelineParameterBufferType,
                        sizeof(params), 1, &params, &buf);
    if (st != VA_STATUS_SUCCESS) {
        fprintf(stderr, "FAIL: vpp CreateBuffer -> %d\n", st);
        *status = 1;
        return;
    }
    st = vaRenderPicture(dpy, ctx, &buf, 1);
    if (st != VA_STATUS_SUCCESS) {
        fprintf(stderr, "FAIL: vpp RenderPicture -> %d\n", st);
        *status = 1;
    }
    st = vaEndPicture(dpy, ctx);
    if (st != VA_STATUS_SUCCESS) {
        fprintf(stderr, "FAIL: vpp EndPicture -> %d\n", st);
        *status = 1;
    }
    vaDestroyBuffer(dpy, buf);
}

int main(int argc, char **argv)
{
    int w = 320, h = 256, nframes = 8, i, status = 0;
    int nofill = 0, nodump = 0, quiet = 0;
    int fillonce = 0;
    int noise = 0;
    int depth = 1;
    int stomp = 0;
    const char *out_path = NULL;
    int drmfd, major = 0, minor = 0, rgb_mode;
    VADisplay dpy;
    VAStatus st;
    VAConfigID vpp_cfg, enc_cfg;
    VAContextID vpp_ctx, enc_ctx;
    VASurfaceID in_surf[MTKVCP_HARN_DEPTH], out_surf[MTKVCP_HARN_DEPTH];
    VABufferID coded_id[MTKVCP_HARN_DEPTH];
    int stride, size, fd[MTKVCP_HARN_DEPTH];
    uint8_t *map[MTKVCP_HARN_DEPTH];
    FILE *ofp;
    size_t total = 0;
    uint64_t t_start, t_end;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--w") && i + 1 < argc)
            w = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--h") && i + 1 < argc)
            h = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--frames") && i + 1 < argc)
            nframes = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--nofill"))
            nofill = 1;
        else if (!strcmp(argv[i], "--fillonce"))
            fillonce = 1;
        else if (!strcmp(argv[i], "--noise"))
            noise = 1;
        /* Simulate the compositor reusing its dma-buf while the encoder is
         * still reading it: rewrite the source right after EndPicture. */
        else if (!strcmp(argv[i], "--stomp"))
            stomp = 1;
        else if (!strcmp(argv[i], "--depth") && i + 1 < argc)
            depth = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--nodump"))
            nodump = 1;
        else if (!strcmp(argv[i], "--quiet"))
            quiet = 1;
        else
            out_path = argv[i];
    }
    if (!out_path || w <= 0 || h <= 0 || nframes <= 0) {
        fprintf(stderr, "usage: %s [--w W] [--h H] [--frames N] "
                "[--depth D] <out.h264>\n",
                argv[0]);
        return 2;
    }
    if (depth < 1)
        depth = 1;
    if (depth > MTKVCP_HARN_DEPTH)
        depth = MTKVCP_HARN_DEPTH;
    for (i = 0; i < MTKVCP_HARN_DEPTH; i++) {
        in_surf[i] = out_surf[i] = VA_INVALID_ID;
        coded_id[i] = VA_INVALID_ID;
        fd[i] = -1;
        map[i] = MAP_FAILED;
    }
    {
        const char *v = getenv("MTK_VCP_VA_RGB_INPUT");
        rgb_mode = v && !strcmp(v, "abgr32") ? 1 :
                   (v && !strcmp(v, "argb32") ? 2 : 0);
    }
    fprintf(stderr, "rgb_input_mode=%d (0=nv12,1=abgr32,2=argb32)\n",
            rgb_mode);

    ofp = fopen(out_path, "wb");
    if (!ofp)
        DIE("open %s", out_path);

    drmfd = open("/dev/dri/renderD128", O_RDWR);
    if (drmfd < 0)
        DIE("open render node");
    dpy = vaGetDisplayDRM(drmfd);
    if (!dpy)
        DIE("vaGetDisplayDRM");
    st = vaInitialize(dpy, &major, &minor);
    CHECKST("vaInitialize", st);

    /* The compositor buffer: 32-bit B,G,R,X, rows pitched to the width.
     * One dma-buf per in-flight slot: the encoder keeps a submitted frame
     * until its coded bytes come back, so a deeper pipeline needs distinct
     * source objects rather than one buffer rewritten underneath it. */
    stride = ((w + 15) & ~15) * 4;
    size = stride * h;
    for (i = 0; i < depth; i++) {
        fd[i] = dmabuf_new((size_t)size);
        if (fd[i] < 0)
            DIE("buffer alloc %d", i);
        map[i] = mmap(NULL, (size_t)size, PROT_READ | PROT_WRITE,
                      MAP_SHARED, fd[i], 0);
        if (map[i] == MAP_FAILED)
            DIE("mmap %d", i);
        /* --fillonce writes the pattern a single time before the loop. The
         * frame-to-frame fill is harness work that would otherwise be
         * counted as chain cost, but leaving the buffer at zero would let
         * the encoder compress trivial content and understate the
         * firmware's own work. */
        if (fillonce)
            noise ? fill_noise(map[i], w, h, stride, i)
                  : fill_bgrx(map[i], w, h, stride, i);
    }
    fprintf(stderr, "buffer fd=%d (%s) depth=%d\n", fd[0],
            fd_is_dmabuf(fd[0]) ? "dma-buf" : "memfd fallback", depth);

    {
        VADRMPRIMESurfaceDescriptor desc;
        VASurfaceAttrib attribs[2];

        for (i = 0; i < depth; i++) {
            memset(&desc, 0, sizeof(desc));
            desc.width = (uint32_t)w;
            desc.height = (uint32_t)h;
            desc.num_objects = 1;
            desc.objects[0].fd = fd[i];
            desc.objects[0].size = (uint32_t)size;
            desc.objects[0].drm_format_modifier = DRM_FORMAT_MOD_LINEAR;
            desc.num_layers = 1;
            desc.layers[0].drm_format = DRM_FORMAT_ARGB8888;
            desc.layers[0].num_planes = 1;
            desc.layers[0].object_index[0] = 0;
            desc.layers[0].offset[0] = 0;
            desc.layers[0].pitch[0] = (uint32_t)stride;
            attribs[0].type = VASurfaceAttribMemoryType;
            attribs[0].flags = VA_SURFACE_ATTRIB_SETTABLE;
            attribs[0].value.type = VAGenericValueTypeInteger;
            attribs[0].value.value.i =
                VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2;
            attribs[1].type = VASurfaceAttribExternalBufferDescriptor;
            attribs[1].flags = VA_SURFACE_ATTRIB_SETTABLE;
            attribs[1].value.type = VAGenericValueTypePointer;
            attribs[1].value.value.p = &desc;
            st = vaCreateSurfaces(dpy, VA_RT_FORMAT_RGB32, w, h,
                                  &in_surf[i], 1, attribs, 2);
            CHECKST("import ARGB8888 dma-buf", st);
        }
    }

    st = vaCreateConfig(dpy, VAProfileNone, VAEntrypointVideoProc, NULL, 0,
                        &vpp_cfg);
    CHECKST("vaCreateConfig(vpp)", st);
    st = vaCreateSurfaces(dpy, VA_RT_FORMAT_YUV420, w, h, out_surf, depth,
                          NULL, 0);
    CHECKST("vaCreateSurfaces(out)", st);
    st = vaCreateContext(dpy, vpp_cfg, w, h, VA_PROGRESSIVE, NULL, 0,
                         &vpp_ctx);
    CHECKST("vaCreateContext(vpp)", st);

    st = vaCreateConfig(dpy, VAProfileH264ConstrainedBaseline,
                        VAEntrypointEncSlice, NULL, 0, &enc_cfg);
    CHECKST("vaCreateConfig(enc)", st);
    st = vaCreateContext(dpy, enc_cfg, w, h, VA_PROGRESSIVE, out_surf, depth,
                         &enc_ctx);
    CHECKST("vaCreateContext(enc)", st);

    /* Sliding window: keep up to `depth` pictures submitted and only wait
     * once the window is full, then collect the oldest. With --depth 1 this
     * is the original submit-then-wait behaviour. */
    t_start = harn_now_us();
    for (i = 0; i < nframes + depth - 1; i++) {
        if (i < nframes) {
            int s_i = i % depth;
            VABufferID seq = VA_INVALID_ID, pic = VA_INVALID_ID;
            VABufferID slice = VA_INVALID_ID, coded = VA_INVALID_ID;
            VAEncSequenceParameterBufferH264 s;
            VAEncPictureParameterBufferH264 p;
            VAEncSliceParameterBufferH264 sp;
            int is_i = (i == 0);

            /* --nofill keeps the buffer untouched: the fill is harness
             * work, not chain work, and at 2460x1080 it costs more than
             * the frame itself, so it has to be removable when timing. */
            if (!nofill && !fillonce)
                noise ? fill_noise(map[s_i], w, h, stride, i)
                      : fill_bgrx(map[s_i], w, h, stride, i);
            run_vpp(dpy, vpp_ctx, in_surf[s_i], out_surf[s_i], &status);
            if (status)
                break;

            st = vaBeginPicture(dpy, enc_ctx, out_surf[s_i]);
            CHECKST("enc BeginPicture", st);
            memset(&s, 0, sizeof(s));
            s.intra_period = 30;
            s.intra_idr_period = 30;
            s.bits_per_second = 2000000;
            st = vaCreateBuffer(dpy, enc_ctx,
                                VAEncSequenceParameterBufferType,
                                sizeof(s), 1, &s, &seq);
            CHECKST("seq", st);
            st = vaRenderPicture(dpy, enc_ctx, &seq, 1);
            CHECKST("render seq", st);
            st = vaCreateBuffer(dpy, enc_ctx, VAEncCodedBufferType,
                                1 << 20, 1, NULL, &coded);
            CHECKST("coded", st);
            memset(&p, 0, sizeof(p));
            p.coded_buf = coded;
            st = vaCreateBuffer(dpy, enc_ctx,
                                VAEncPictureParameterBufferType,
                                sizeof(p), 1, &p, &pic);
            CHECKST("pic", st);
            st = vaRenderPicture(dpy, enc_ctx, &pic, 1);
            CHECKST("render pic", st);
            memset(&sp, 0, sizeof(sp));
            sp.slice_type = is_i ? 2 : 0;
            sp.num_macroblocks = (uint32_t)(w / 16) * (uint32_t)(h / 16);
            if (is_i)
                sp.idr_pic_id = 1;
            st = vaCreateBuffer(dpy, enc_ctx,
                                VAEncSliceParameterBufferType,
                                sizeof(sp), 1, &sp, &slice);
            CHECKST("slice", st);
            st = vaRenderPicture(dpy, enc_ctx, &slice, 1);
            CHECKST("render slice", st);
            st = vaEndPicture(dpy, enc_ctx);
            CHECKST("enc EndPicture", st);
            if (stomp)
                noise ? fill_noise(map[s_i], w, h, stride, i + 1000)
                      : fill_bgrx(map[s_i], w, h, stride, i + 1000);
            vaDestroyBuffer(dpy, seq);
            vaDestroyBuffer(dpy, pic);
            vaDestroyBuffer(dpy, slice);
            coded_id[s_i] = coded;
        }

        /* Collect one picture once the window is full, and drain the rest
         * at the end so every submitted frame is still verified. */
        if (i >= depth - 1) {
            int c_i = (i - (depth - 1)) % depth;

            if (harn_collect(dpy, coded_id[c_i], ofp, nodump, &total) < 0) {
                status = 1;
                break;
            }
            vaDestroyBuffer(dpy, coded_id[c_i]);
            coded_id[c_i] = VA_INVALID_ID;
            if (!quiet && ((i - (depth - 1)) % 30) == 0)
                fprintf(stderr, "collected frame %d\n", i - (depth - 1));
        }
    }
    t_end = harn_now_us();
    if (!status) {
        double secs = (double)(t_end - t_start) / 1e6;

        fprintf(stderr, "HARN depth=%d frames=%d wall=%.3fs -> %.1f fps\n",
                depth, nframes, secs, (double)nframes / secs);
    }
    fclose(ofp);

    for (i = 0; i < depth; i++) {
        if (coded_id[i] != VA_INVALID_ID)
            vaDestroyBuffer(dpy, coded_id[i]);
        if (in_surf[i] != VA_INVALID_ID)
            vaDestroySurfaces(dpy, &in_surf[i], 1);
        if (map[i] != MAP_FAILED)
            munmap(map[i], (size_t)size);
        if (fd[i] >= 0)
            close(fd[i]);
    }
    vaDestroyContext(dpy, enc_ctx);
    vaDestroyContext(dpy, vpp_ctx);
    vaDestroySurfaces(dpy, out_surf, depth);
    vaDestroyConfig(dpy, enc_cfg);
    vaDestroyConfig(dpy, vpp_cfg);
    vaTerminate(dpy);
    close(drmfd);
    if (status)
        return 1;
    printf("RGBCHAIN-OK frames=%d bytes=%zu\n", nframes, total);
    return 0;
}
