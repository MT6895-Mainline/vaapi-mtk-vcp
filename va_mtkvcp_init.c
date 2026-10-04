/* SPDX-License-Identifier: MIT */
/* Driver init, profiles, configs, contexts, buffers, picture dispatch. */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <sys/syscall.h>

#include <va/va.h>
#include <va/va_backend.h>
#include <va/va_backend_vpp.h>
#include <va/va_vpp.h>
#include <va/va_enc_h264.h>
#include <va/va_enc_hevc.h>
#include <va/va_drmcommon.h>

#include "va_mtkvcp.h"

/* ---- logging ---- */
static int mtkvcp_debug_enabled(void)
{
    static int cached = -1;
    if (cached < 0)
        cached = getenv("MTK_VCP_VA_DEBUG") ? 1 : 0;
    return cached;
}

void mtkvcp_log(const char *fmt, ...)
{
    va_list ap;
    if (!mtkvcp_debug_enabled())
        return;
    fprintf(stderr, "mtk-vcp-va: [%llu] ",
            (unsigned long long)mtkvcp_now_us());
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fprintf(stderr, "\n");
}

/* ---- capture-path profiling ----
 *
 * The whole point of this block is to attribute the frame cost to a stage
 * instead of guessing. The achieved rate says the chain is slow; only the
 * split says whether that is the VideoProc copy, the dma-buf probe, the
 * submit ioctl, or the wait for the coded frame to come back.
 */
uint64_t mtkvcp_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

int mtkvcp_prof_enabled(void)
{
    static int cached = -1;
    if (cached < 0)
        cached = getenv("MTK_VCP_VA_PROF") ? 1 : 0;
    return cached;
}

/*
 * Profiling output has to be independent of MTK_VCP_VA_DEBUG. Turning the
 * debug log on to see the report would make the driver print several lines
 * per frame, and at 60 fps that formatting and the journald write are a
 * measurable part of the very cost being measured. MTK_VCP_VA_PROF alone
 * emits only the once-a-second report, and MTK_VCP_VA_PROF_FILE sends it to
 * a file instead of stderr so a background service does not have to keep a
 * journal around for it.
 */
static FILE *mtkvcp_prof_stream(void)
{
    static FILE *fp;
    static int tried;

    if (!tried) {
        const char *path = getenv("MTK_VCP_VA_PROF_FILE");
        tried = 1;
        if (path && *path)
            fp = fopen(path, "a");
        if (!fp)
            fp = stderr;
    }
    return fp;
}

static const char *const mtkvcp_prof_name[MTKVCP_PROF_STAGES] = {
    "vpp_total", "vpp_probe", "vpp_map", "vpp_dup",
    "enc_reap", "enc_copy", "enc_qbuf", "enc_wait", "enc_recycle",
    "enc_total",
    "enc_inwait",
};

void mtkvcp_prof_stage(struct mtkvcp_context *c, int which, uint64_t us)
{
    if (!mtkvcp_prof_enabled() || which < 0 || which >= MTKVCP_PROF_STAGES)
        return;
    c->prof_us[which] += us;
    if (us > c->prof_max_us[which])
        c->prof_max_us[which] = us;
}

/* Emit one line per stage once a second, then reset the window. Called from
 * the encoder at the end of each frame so the report lines up with frames
 * rather than with wall-clock time.
 */
static void mtkvcp_prof_report(struct mtkvcp_context *c, const char *tag)
{
    uint64_t now, span;
    FILE *fp;
    int i;

    if (!mtkvcp_prof_enabled())
        return;
    now = mtkvcp_now_us() / 1000ull;
    if (!c->prof_last_report_ms) {
        c->prof_last_report_ms = now;
        return;
    }
    span = now - c->prof_last_report_ms;
    if (span < 1000)
        return;
    if (!c->prof_frames) {
        c->prof_last_report_ms = now;
        return;
    }
    fp = mtkvcp_prof_stream();
    /* The thread id matters on the real krdp path: the capture pipeline
     * runs on its own thread, so knowing which one is encoding says
     * whether the RDP transport is competing with the encoder for a core. */
    fprintf(fp, "PROF[%s] tid=%d frames=%llu span=%llums -> %.1f fps\n",
            tag, (int)syscall(SYS_gettid),
            (unsigned long long)c->prof_frames,
            (unsigned long long)span,
            (double)c->prof_frames * 1000.0 / (double)span);
    for (i = 0; i < MTKVCP_PROF_STAGES; i++) {
        if (!c->prof_us[i] && !c->prof_max_us[i])
            continue;
        fprintf(fp, "PROF[%s]   %-12s avg=%7.2fms max=%7.2fms\n",
                tag, mtkvcp_prof_name[i],
                (double)c->prof_us[i] / 1000.0 / (double)c->prof_frames,
                (double)c->prof_max_us[i] / 1000.0);
    }
    fflush(fp);
    for (i = 0; i < MTKVCP_PROF_STAGES; i++) {
        c->prof_us[i] = 0;
        c->prof_max_us[i] = 0;
    }
    c->prof_frames = 0;
    c->prof_last_report_ms = now;
}

void mtkvcp_prof_frame(struct mtkvcp_context *c, const char *tag);

void mtkvcp_prof_frame(struct mtkvcp_context *c, const char *tag)
{
    if (!mtkvcp_prof_enabled())
        return;
    c->prof_frames++;
    mtkvcp_prof_report(c, tag);
}

/*
 * Packed-RGB capture path.
 *
 * The VCP encoder firmware takes packed 32-bit RGB as a raw input and runs
 * the RGB-to-YUV conversion itself (verified on hardware: V4L2_PIX_FMT_
 * ABGR32 reaches it as venc_yuv_fmt 16, ARGB32 as 15). The krdp/KPipeWire
 * capture chain hands the compositor's DRM_FORMAT_ARGB8888 dma-buf to
 * scale_vaapi, which used to convert it to NV12 on the CPU. With this
 * enabled the post-processing step carries the RGB bytes through unchanged
 * and the encoder submits them as packed RGB, so the conversion happens in
 * the firmware.
 *
 * No VA-API client can express "my encoder input is packed RGB": ffmpeg
 * derives the surface format from the profile (YUV420) and h264_vaapi only
 * accepts YUV render targets. The switch is therefore environmental and
 * belongs in the environment of the process that drives the capture chain
 * (see the krdpserver launcher), not in the stream.
 *
 * MTK_VCP_VA_RGB_INPUT=abgr32  -> bytes B,G,R,X (DRM_FORMAT_ARGB8888)
 * MTK_VCP_VA_RGB_INPUT=argb32  -> bytes A,R,G,B
 * unset/0/off                  -> NV12 as before
 */
int mtkvcp_rgb_input_mode(void)
{
    static int cached = -1;
    const char *v;
    if (cached >= 0)
        return cached;
    cached = MTKVCP_RGB_NONE;
    v = getenv("MTK_VCP_VA_RGB_INPUT");
    if (v && *v) {
        if (!strcmp(v, "abgr32") || !strcmp(v, "ar24") ||
            !strcmp(v, "argb8888") || !strcmp(v, "1"))
            cached = MTKVCP_RGB_ABGR32;
        else if (!strcmp(v, "argb32") || !strcmp(v, "ba24"))
            cached = MTKVCP_RGB_ARGB32;
        else if (strcmp(v, "0") && strcmp(v, "off") && strcmp(v, "nv12"))
            fprintf(stderr, "mtk-vcp-va: MTK_VCP_VA_RGB_INPUT=%s not "
                    "recognised, using NV12\n", v);
    }
    return cached;
}

/* Handing the compositor's own dma-buf to the encoder is opt-in, because
 * it cannot be made safe from inside the driver.
 *
 * PipeWireSourceStream::process() returns the pw_buffer to the producer as
 * soon as handleFrame() returns, and handleFrame() only queues the frame
 * into the filter graph -- it does not wait for the VPP. KPipeWire keeps up
 * to m_maxPendingFrames (50) frames in that queue. So the compositor is
 * free to draw the next frame into the very memory the firmware is still
 * reading, and a moving desktop shows it as a torn frame. The kernel does
 * signal when the firmware releases the input (the OUTPUT buffer comes
 * back), but by then the producer has already rewritten the buffer.
 *
 * The default is therefore the copy path: the VPP reads the compositor's
 * buffer inside EndPicture and the encoder works from driver-owned memory,
 * which no producer can touch. Set MTK_VCP_VA_ZEROCOPY=1 to opt back in for
 * callers that own their buffers for the whole frame (the harness does).
 */
int mtkvcp_zerocopy_enabled(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *v = getenv("MTK_VCP_VA_ZEROCOPY");

        cached = 0;
        if (v && (!strcmp(v, "1") || !strcmp(v, "on") ||
                  !strcmp(v, "yes")))
            cached = 1;
    }
    return cached;
}

/* The V4L2 OUTPUT fourcc that carries the selected packed-RGB layout. */
unsigned int mtkvcp_rgb_fourcc(int mode)
{
    switch (mode) {
    case MTKVCP_RGB_ABGR32: return V4L2_PIX_FMT_ABGR32;
    case MTKVCP_RGB_ARGB32: return V4L2_PIX_FMT_ARGB32;
    default: return 0;
    }
}

/* Row pitch of the driver's packed-RGB source layout. vcp_venc_calc_layout
 * uses ALIGN(width, 16) samples per row at four bytes each, and the public
 * bytesperline S_FMT reports is the same value, so both sides agree.
 */
int mtkvcp_rgb_stride(int width)
{
    return ((width + 15) & ~15) * 4;
}

const char *mtkvcp_va_status_str(int status)
{
    switch (status) {
    case VA_STATUS_SUCCESS: return "SUCCESS";
    case VA_STATUS_ERROR_HW_BUSY: return "HW_BUSY";
    case VA_STATUS_ERROR_ALLOCATION_FAILED: return "ALLOCATION_FAILED";
    case VA_STATUS_ERROR_INVALID_SURFACE: return "INVALID_SURFACE";
    case VA_STATUS_ERROR_INVALID_CONTEXT: return "INVALID_CONTEXT";
    case VA_STATUS_ERROR_INVALID_CONFIG: return "INVALID_CONFIG";
    case VA_STATUS_ERROR_INVALID_BUFFER: return "INVALID_BUFFER";
    case VA_STATUS_ERROR_INVALID_IMAGE: return "INVALID_IMAGE";
    case VA_STATUS_ERROR_INVALID_VALUE: return "INVALID_VALUE";
    case VA_STATUS_ERROR_INVALID_PARAMETER: return "INVALID_PARAMETER";
    case VA_STATUS_ERROR_UNSUPPORTED_PROFILE: return "UNSUPPORTED_PROFILE";
    case VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT: return "UNSUPPORTED_ENTRYPOINT";
    case VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT: return "UNSUPPORTED_RT_FORMAT";
    case VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE: return "UNSUPPORTED_BUFFERTYPE";
    case VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED: return "RESOLUTION_NOT_SUPPORTED";
    case VA_STATUS_ERROR_DECODING_ERROR: return "DECODING_ERROR";
    case VA_STATUS_ERROR_ENCODING_ERROR: return "ENCODING_ERROR";
    case VA_STATUS_ERROR_UNIMPLEMENTED: return "UNIMPLEMENTED";
    case VA_STATUS_ERROR_ATTR_NOT_SUPPORTED: return "ATTR_NOT_SUPPORTED";
    default: return "?";
    }
}

/* ---- profile table: only verified decoder/encoder paths ---- */
struct mtkvcp_prof {
    int profile;
    int entrypoint;
    unsigned int rt_format;
    uint32_t out_fourcc;  /* V4L2: coded-in (dec) / raw-in (enc) */
    uint32_t cap_fourcc;  /* V4L2: raw-out (dec) / coded-out (enc) */
    int is_encode;
    int is_10bit;
    int is_vpp;           /* CPU post-proc, no V4L2 session */
};

static const struct mtkvcp_prof mtkvcp_profiles[] = {
    { VAProfileH264ConstrainedBaseline, VAEntrypointVLD, VA_RT_FORMAT_YUV420,
      V4L2_PIX_FMT_H264,       V4L2_PIX_FMT_NV12, 0, 0, 0 },
    { VAProfileH264Main,     VAEntrypointVLD, VA_RT_FORMAT_YUV420,
      V4L2_PIX_FMT_H264,       V4L2_PIX_FMT_NV12, 0, 0, 0 },
    { VAProfileH264High,     VAEntrypointVLD, VA_RT_FORMAT_YUV420,
      V4L2_PIX_FMT_H264,       V4L2_PIX_FMT_NV12, 0, 0, 0 },
    { VAProfileHEVCMain,     VAEntrypointVLD, VA_RT_FORMAT_YUV420,
      V4L2_PIX_FMT_HEVC,       V4L2_PIX_FMT_NV12, 0, 0, 0 },
    { VAProfileHEVCMain10,   VAEntrypointVLD, VA_RT_FORMAT_YUV420_10,
      V4L2_PIX_FMT_HEVC,       V4L2_PIX_FMT_P010, 0, 1, 0 },
    { VAProfileVP9Profile0,  VAEntrypointVLD, VA_RT_FORMAT_YUV420,
      V4L2_PIX_FMT_VP9,       V4L2_PIX_FMT_NV12, 0, 0, 0 },
    { VAProfileMPEG2Simple,  VAEntrypointVLD, VA_RT_FORMAT_YUV420,
      V4L2_PIX_FMT_MPEG2,       V4L2_PIX_FMT_NV12, 0, 0, 0 },
    { VAProfileMPEG2Main,    VAEntrypointVLD, VA_RT_FORMAT_YUV420,
      V4L2_PIX_FMT_MPEG2,       V4L2_PIX_FMT_NV12, 0, 0, 0 },
    /* Encode: CBR only (firmware VBR is a no-op, stays unexposed). */
    /* ConstrainedBaseline is what krdp/KPipeWire asks for (it maps its
     * "H264Baseline" encoder onto AV_PROFILE_H264_CONSTRAINED_BASELINE,
     * and ffmpeg then needs an EncSlice entrypoint for VAProfile 13).
     * The firmware emits profile_idc 66 for it; see the profile control
     * in mtkvcp_enc_create. */
    { VAProfileH264ConstrainedBaseline, VAEntrypointEncSlice,
      VA_RT_FORMAT_YUV420,
      V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_H264, 1, 0, 0 },
    { VAProfileH264Main,     VAEntrypointEncSlice, VA_RT_FORMAT_YUV420,
      V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_H264, 1, 0, 0 },
    { VAProfileH264High,     VAEntrypointEncSlice, VA_RT_FORMAT_YUV420,
      V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_H264, 1, 0, 0 },
    { VAProfileHEVCMain,     VAEntrypointEncSlice, VA_RT_FORMAT_YUV420,
      V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_HEVC, 1, 0, 0 },
    /* VideoProc: CPU convert/scale for the krdp hwmap+scale_vaapi path. */
    { VAProfileNone,         VAEntrypointVideoProc,
      VA_RT_FORMAT_YUV420 | VA_RT_FORMAT_RGB32,
      0, 0, 0, 0, 1 },
};

static const struct mtkvcp_prof *mtkvcp_find_prof(int profile, int entrypoint)
{
    unsigned int i;
    for (i = 0; i < sizeof(mtkvcp_profiles) / sizeof(mtkvcp_profiles[0]); i++)
        if (mtkvcp_profiles[i].profile == profile &&
            mtkvcp_profiles[i].entrypoint == entrypoint)
            return &mtkvcp_profiles[i];
    return NULL;
}

static struct mtkvcp_drv *mtkvcp_drv_of(VADriverContextP ctx)
{
    return (struct mtkvcp_drv *)ctx->pDriverData;
}

/* id helpers: 0 is VA_INVALID_ID, slots are index+1 */
static int mtkvcp_alloc_config(struct mtkvcp_drv *d)
{
    int i;
    for (i = 0; i < MTKVCP_MAX_CONTEXTS; i++)
        if (!d->configs[i].in_use) {
            d->configs[i].in_use = 1;
            return i;
        }
    return -1;
}

static int mtkvcp_alloc_context(struct mtkvcp_drv *d)
{
    int i;
    for (i = 0; i < MTKVCP_MAX_CONTEXTS; i++)
        if (!d->contexts[i].in_use) {
            d->contexts[i].in_use = 1;
            return i;
        }
    return -1;
}

static int mtkvcp_alloc_surface(struct mtkvcp_drv *d)
{
    int i;
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
        if (!d->surfaces[i].in_use) {
            d->surfaces[i].in_use = 1;
            return i;
        }
    return -1;
}

static int mtkvcp_alloc_buffer(struct mtkvcp_drv *d)
{
    int i;
    for (i = 0; i < MTKVCP_MAX_BUFFERS; i++)
        if (!d->buffers[i].in_use) {
            d->buffers[i].in_use = 1;
            return i;
        }
    return -1;
}

/* ---- vtable forward declarations (implemented across files) ---- */
static VAStatus mtkvcp_Terminate(VADriverContextP ctx);
static VAStatus mtkvcp_QueryConfigProfiles(VADriverContextP ctx,
    VAProfile *list, int *num);
static VAStatus mtkvcp_QueryConfigEntrypoints(VADriverContextP ctx,
    VAProfile profile, VAEntrypoint *list, int *num);
static VAStatus mtkvcp_GetConfigAttributes(VADriverContextP ctx,
    VAProfile profile, VAEntrypoint entrypoint,
    VAConfigAttrib *attribs, int num_attribs);
static VAStatus mtkvcp_CreateConfig(VADriverContextP ctx, VAProfile profile,
    VAEntrypoint entrypoint, VAConfigAttrib *attribs, int num_attribs,
    VAConfigID *config_id);
static VAStatus mtkvcp_DestroyConfig(VADriverContextP ctx, VAConfigID id);
static VAStatus mtkvcp_QueryConfigAttributes(VADriverContextP ctx,
    VAConfigID id, VAProfile *profile, VAEntrypoint *entrypoint,
    VAConfigAttrib *attribs, int *num_attribs);
static VAStatus mtkvcp_CreateSurfaces(VADriverContextP ctx, int w, int h,
    int format, int n, VASurfaceID *surfaces);
static VAStatus mtkvcp_DestroySurfaces(VADriverContextP ctx,
    VASurfaceID *list, int n);
static VAStatus mtkvcp_CreateSurfaces2(VADriverContextP ctx,
    unsigned int format, unsigned int w, unsigned int h,
    VASurfaceID *surfaces, unsigned int n,
    VASurfaceAttrib *attribs, unsigned int num_attribs);
static VAStatus mtkvcp_QuerySurfaceAttributes(VADriverContextP ctx,
    VAConfigID config, VASurfaceAttrib *attribs, unsigned int *num);
static VAStatus mtkvcp_CreateContext(VADriverContextP ctx, VAConfigID cfg,
    int w, int h, int flag, VASurfaceID *targets, int num_targets,
    VAContextID *context);
static VAStatus mtkvcp_DestroyContext(VADriverContextP ctx, VAContextID id);
static VAStatus mtkvcp_CreateBuffer(VADriverContextP ctx, VAContextID c,
    VABufferType type, unsigned int size, unsigned int num_elements,
    void *data, VABufferID *buf_id);
static VAStatus mtkvcp_BufferSetNumElements(VADriverContextP ctx,
    VABufferID id, unsigned int n);
static VAStatus mtkvcp_MapBuffer(VADriverContextP ctx, VABufferID id,
    void **pbuf);
static VAStatus mtkvcp_UnmapBuffer(VADriverContextP ctx, VABufferID id);
static VAStatus mtkvcp_DestroyBuffer(VADriverContextP ctx, VABufferID id);
static VAStatus mtkvcp_BeginPicture(VADriverContextP ctx, VAContextID c,
    VASurfaceID target);
static VAStatus mtkvcp_RenderPicture(VADriverContextP ctx, VAContextID c,
    VABufferID *buffers, int num_buffers);
static VAStatus mtkvcp_EndPicture(VADriverContextP ctx, VAContextID c);
static VAStatus mtkvcp_SyncSurface(VADriverContextP ctx, VASurfaceID s);
static VAStatus mtkvcp_SyncSurface2(VADriverContextP ctx, VASurfaceID s,
    uint64_t timeout_ns);
static VAStatus mtkvcp_QuerySurfaceStatus(VADriverContextP ctx,
    VASurfaceID s, VASurfaceStatus *status);
static VAStatus mtkvcp_PutSurface(VADriverContextP ctx, VASurfaceID s,
    void *draw, short srcx, short srcy, unsigned short srcw,
    unsigned short srch, short destx, short desty, unsigned short destw,
    unsigned short desth, VARectangle *clips, unsigned int nclips,
    unsigned int flags);
VAStatus mtkvcp_QueryImageFormats(VADriverContextP ctx,
    VAImageFormat *formats, int *num);
VAStatus mtkvcp_CreateImage(VADriverContextP ctx, VAImageFormat *f,
    int w, int h, VAImage *image);
VAStatus mtkvcp_DeriveImage(VADriverContextP ctx, VASurfaceID s,
    VAImage *image);
VAStatus mtkvcp_DestroyImage(VADriverContextP ctx, VAImageID id);
VAStatus mtkvcp_GetImage(VADriverContextP ctx, VASurfaceID s, int x,
    int y, unsigned int w, unsigned int h, VAImageID id);
VAStatus mtkvcp_PutImage(VADriverContextP ctx, VASurfaceID s,
    VAImageID id, int sx, int sy, unsigned int sw, unsigned int sh,
    int dx, int dy, unsigned int dw, unsigned int dh);
static VAStatus mtkvcp_BufferInfo(VADriverContextP ctx, VABufferID id,
    VABufferType *type, unsigned int *size, unsigned int *num_elements);
VAStatus mtkvcp_ExportSurfaceHandle(VADriverContextP ctx,
    VASurfaceID s, uint32_t mem_type, uint32_t flags, void *desc);
VAStatus mtkvcp_AcquireBufferHandle(VADriverContextP ctx, VABufferID id,
    VABufferInfo *info);
VAStatus mtkvcp_ReleaseBufferHandle(VADriverContextP ctx, VABufferID id);
static VAStatus mtkvcp_SyncBuffer(VADriverContextP ctx, VABufferID id,
    uint64_t timeout_ns);
static VAStatus mtkvcp_MapBuffer2(VADriverContextP ctx, VABufferID id,
    void **pbuf, uint32_t flags);
static VAStatus mtkvcp_QueryDisplayAttributes(VADriverContextP ctx,
    VADisplayAttribute *attrs, int *num);
static VAStatus mtkvcp_GetDisplayAttributes(VADriverContextP ctx,
    VADisplayAttribute *attrs, int num);
static VAStatus mtkvcp_SetDisplayAttributes(VADriverContextP ctx,
    VADisplayAttribute *attrs, int num);

/* decode/encode cores live in their own files; vpp core too */
VAStatus mtkvcp_dec_create(struct mtkvcp_drv *d, int ci);
void mtkvcp_dec_destroy(struct mtkvcp_drv *d, int ci);
VAStatus mtkvcp_dec_begin(struct mtkvcp_drv *d, int ci, int si);
VAStatus mtkvcp_dec_render(struct mtkvcp_drv *d, int ci, int bi);
VAStatus mtkvcp_dec_end(struct mtkvcp_drv *d, int ci);
VAStatus mtkvcp_dec_sync(struct mtkvcp_drv *d, int si, uint64_t timeout_ns);
VAStatus mtkvcp_enc_create(struct mtkvcp_drv *d, int ci);
void mtkvcp_enc_destroy(struct mtkvcp_drv *d, int ci);
VAStatus mtkvcp_enc_begin(struct mtkvcp_drv *d, int ci, int si);
VAStatus mtkvcp_enc_render(struct mtkvcp_drv *d, int ci, int bi);
VAStatus mtkvcp_enc_end(struct mtkvcp_drv *d, int ci);
VAStatus mtkvcp_vpp_begin(struct mtkvcp_drv *d, int ci, int si);
VAStatus mtkvcp_vpp_render(struct mtkvcp_drv *d, int ci, int bi);
VAStatus mtkvcp_vpp_end(struct mtkvcp_drv *d, int ci);
VAStatus mtkvcp_vpp_query_filters(VADriverContextP ctx,
    VAContextID context, VAProcFilterType *filters,
    unsigned int *num_filters);
VAStatus mtkvcp_vpp_query_filter_caps(VADriverContextP ctx,
    VAContextID context, VAProcFilterType type, void *caps,
    unsigned int *num_caps);
VAStatus mtkvcp_vpp_query_pipeline_caps(VADriverContextP ctx,
    VAContextID context, VABufferID *filters, unsigned int num_filters,
    VAProcPipelineCaps *caps);

static VAStatus mtkvcp_not_supported(VADriverContextP ctx)
{
    (void)ctx;
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

static VAStatus mtkvcp_Terminate(VADriverContextP ctx)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int i;
    if (!d)
        return VA_STATUS_SUCCESS;
    mtkvcp_log("terminate");
    free(ctx->vtable_vpp);
    ctx->vtable_vpp = NULL;
    pthread_mutex_lock(&d->lock);
    for (i = 0; i < MTKVCP_MAX_CONTEXTS; i++)
        if (d->contexts[i].in_use) {
            if (d->contexts[i].is_encode)
                mtkvcp_enc_destroy(d, i);
            else
                mtkvcp_dec_destroy(d, i);
            d->contexts[i].in_use = 0;
        }
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
        if (d->surfaces[i].in_use)
            mtkvcp_surface_release(d, i);
    for (i = 0; i < MTKVCP_MAX_BUFFERS; i++)
        if (d->buffers[i].in_use) {
            if (d->buffers[i].handle_fd >= 0)
                close(d->buffers[i].handle_fd);
            free(d->buffers[i].data);
            free(d->buffers[i].coded_bytes);
            d->buffers[i].in_use = 0;
        }
    for (i = 0; i < MTKVCP_MAX_IMAGES; i++)
        if (d->images[i].in_use) {
            if (d->images[i].derived_surface < 0)
                free(d->images[i].data);
            d->images[i].in_use = 0;
        }
    pthread_mutex_unlock(&d->lock);
    pthread_mutex_destroy(&d->lock);
    free(d);
    ctx->pDriverData = NULL;
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_QueryConfigProfiles(VADriverContextP ctx,
    VAProfile *list, int *num)
{
    unsigned int i, n = 0;
    int seen[64];
    unsigned int ns = 0, k;
    (void)ctx;
    if (!list || !num)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    for (i = 0; i < sizeof(mtkvcp_profiles) / sizeof(mtkvcp_profiles[0]); i++) {
        int p = mtkvcp_profiles[i].profile;
        for (k = 0; k < ns; k++)
            if (seen[k] == p)
                break;
        if (k == ns)
            seen[ns++] = p;
    }
    for (k = 0; k < ns && n < 64; k++)
        list[n++] = seen[k];
    *num = (int)n;
    mtkvcp_log("profiles -> %d", (int)n);
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_QueryConfigEntrypoints(VADriverContextP ctx,
    VAProfile profile, VAEntrypoint *list, int *num)
{
    unsigned int i, n = 0;
    (void)ctx;
    if (!list || !num)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    for (i = 0; i < sizeof(mtkvcp_profiles) / sizeof(mtkvcp_profiles[0]); i++)
        if (mtkvcp_profiles[i].profile == profile)
            list[n++] = mtkvcp_profiles[i].entrypoint;
    if (!n)
        return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;
    *num = (int)n;
    mtkvcp_log("entrypoints profile=%d -> %d", profile, (int)n);
    return VA_STATUS_SUCCESS;
}

static void mtkvcp_attrib_value(const struct mtkvcp_prof *p, int type,
                                unsigned int *val)
{
    switch (type) {
    case VAConfigAttribRTFormat:
        *val = p->rt_format;
        break;
    case VAConfigAttribMaxPictureWidth:
        *val = MTKVCP_MAX_W;
        break;
    case VAConfigAttribMaxPictureHeight:
        *val = MTKVCP_MAX_H;
        break;
    case VAConfigAttribDecSliceMode:
        *val = p->is_encode ? VA_ATTRIB_NOT_SUPPORTED :
               VA_DEC_SLICE_MODE_NORMAL;
        break;
    case VAConfigAttribEncPackedHeaders:
        *val = p->is_encode ? VA_ENC_PACKED_HEADER_NONE :
               VA_ATTRIB_NOT_SUPPORTED;
        break;
    case VAConfigAttribEncInterlaced:
        *val = p->is_encode ? VA_ENC_INTERLACED_NONE :
               VA_ATTRIB_NOT_SUPPORTED;
        break;
    case VAConfigAttribEncMaxRefFrames:
        /* Firmware B-frame window is 0-2; report the open range. */
        *val = p->is_encode ? 2 : VA_ATTRIB_NOT_SUPPORTED;
        break;
    case VAConfigAttribEncMaxSlices:
        /* Whole frames are submitted at once. */
        *val = p->is_encode ? 1 : VA_ATTRIB_NOT_SUPPORTED;
        break;
    case VAConfigAttribEncSliceStructure:
        *val = p->is_encode ? VA_ENC_SLICE_STRUCTURE_ARBITRARY_ROWS :
               VA_ATTRIB_NOT_SUPPORTED;
        break;
    case VAConfigAttribRateControl:
        /* CBR is the only mode the firmware implements. CQP is also
         * advertised because clients such as KPipeWire ask for quality
         * without a bitrate: ffmpeg then needs CQP (or ICQ), and with
         * CBR alone it refuses outright ("Driver does not support any
         * RC mode compatible with selected options"), which is how the
         * RDP server ended up on libx264. In CQP mode ffmpeg puts the
         * requested QP in the picture params; the encode path reads it
         * and converts it to a CBR target, so the quality control still
         * works. Bitrate-driven clients keep getting real CBR. See
         * mtkvcp_enc_apply_params. */
        *val = p->is_encode ? (VA_RC_CBR | VA_RC_CQP | VA_RC_ICQ) :
               VA_ATTRIB_NOT_SUPPORTED;
        break;
    case VAConfigAttribEncROI:
        *val = p->is_encode ? 0 : VA_ATTRIB_NOT_SUPPORTED;
        break;
    case VAConfigAttribEncMaxTileRows:
    case VAConfigAttribEncMaxTileCols:
    case VAConfigAttribEncTileSupport:
    case VAConfigAttribEncDynamicScaling:
    case VAConfigAttribEncQualityRange:
    case VAConfigAttribEncDirtyRect:
    case VAConfigAttribEncParallelRateControl:
    case VAConfigAttribStats:
    case VAConfigAttribDecProcessing:
    default:
        *val = VA_ATTRIB_NOT_SUPPORTED;
        break;
    }
}

static VAStatus mtkvcp_GetConfigAttributes(VADriverContextP ctx,
    VAProfile profile, VAEntrypoint entrypoint,
    VAConfigAttrib *attribs, int num_attribs)
{
    const struct mtkvcp_prof *p;
    int i;
    (void)ctx;
    p = mtkvcp_find_prof(profile, entrypoint);
    if (!p)
        return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;
    for (i = 0; i < num_attribs; i++)
        mtkvcp_attrib_value(p, attribs[i].type, &attribs[i].value);
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_CreateConfig(VADriverContextP ctx, VAProfile profile,
    VAEntrypoint entrypoint, VAConfigAttrib *attribs, int num_attribs,
    VAConfigID *config_id)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    const struct mtkvcp_prof *p;
    int ci, i;
    unsigned int rt = 0;
    int have_rt = 0;
    if (!config_id)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    p = mtkvcp_find_prof(profile, entrypoint);
    if (!p)
        return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;
    for (i = 0; i < num_attribs; i++) {
        if (attribs[i].type == VAConfigAttribRTFormat) {
            rt = attribs[i].value;
            have_rt = 1;
        }
    }
    if (have_rt && !(rt & p->rt_format))
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    pthread_mutex_lock(&d->lock);
    ci = mtkvcp_alloc_config(d);
    if (ci < 0) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }
    d->configs[ci].profile = profile;
    d->configs[ci].entrypoint = entrypoint;
    d->configs[ci].rt_format = have_rt ? rt : p->rt_format;
    pthread_mutex_unlock(&d->lock);
    *config_id = (VAConfigID)(ci + 1);
    mtkvcp_log("CreateConfig profile=%d entrypoint=%d -> %d", profile,
               entrypoint, ci + 1);
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_DestroyConfig(VADriverContextP ctx, VAConfigID id)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int ci = (int)id - 1;
    if (ci < 0 || ci >= MTKVCP_MAX_CONTEXTS)
        return VA_STATUS_ERROR_INVALID_CONFIG;
    pthread_mutex_lock(&d->lock);
    if (!d->configs[ci].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_CONFIG;
    }
    d->configs[ci].in_use = 0;
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_QueryConfigAttributes(VADriverContextP ctx,
    VAConfigID id, VAProfile *profile, VAEntrypoint *entrypoint,
    VAConfigAttrib *attribs, int *num_attribs)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    static const VAConfigAttribType types[] = {
        VAConfigAttribRTFormat,
        VAConfigAttribMaxPictureWidth, VAConfigAttribMaxPictureHeight,
        VAConfigAttribDecSliceMode,
        VAConfigAttribEncPackedHeaders, VAConfigAttribEncInterlaced,
        VAConfigAttribEncMaxRefFrames, VAConfigAttribEncMaxSlices,
        VAConfigAttribEncSliceStructure, VAConfigAttribRateControl,
    };
    const struct mtkvcp_prof *p;
    unsigned int i, n;
    int ci = (int)id - 1;
    if (!profile || !entrypoint || !attribs || !num_attribs)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (ci < 0 || ci >= MTKVCP_MAX_CONTEXTS)
        return VA_STATUS_ERROR_INVALID_CONFIG;
    pthread_mutex_lock(&d->lock);
    if (!d->configs[ci].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_CONFIG;
    }
    *profile = d->configs[ci].profile;
    *entrypoint = d->configs[ci].entrypoint;
    pthread_mutex_unlock(&d->lock);
    p = mtkvcp_find_prof(*profile, *entrypoint);
    if (!p)
        return VA_STATUS_ERROR_INVALID_CONFIG;
    n = sizeof(types) / sizeof(types[0]);
    if ((unsigned int)*num_attribs < n) {
        *num_attribs = (int)n;
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }
    for (i = 0; i < n; i++) {
        attribs[i].type = types[i];
        mtkvcp_attrib_value(p, types[i], &attribs[i].value);
    }
    *num_attribs = (int)n;
    return VA_STATUS_SUCCESS;
}

/* ---- surfaces ---- */
static VAStatus mtkvcp_create_surfs_locked(struct mtkvcp_drv *d, int w,
    int h, unsigned int rt_format, int n, VASurfaceID *out)
{
    int i, si;
    if (w <= 0 || h <= 0 || n <= 0)
        return VA_STATUS_ERROR_INVALID_VALUE;
    if (rt_format != 0 && rt_format != VA_RT_FORMAT_YUV420 &&
        rt_format != VA_RT_FORMAT_YUV420_10)
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    for (i = 0; i < n; i++) {
        si = mtkvcp_alloc_surface(d);
        if (si < 0) {
            int j;
            for (j = 0; j < i; j++)
                mtkvcp_surface_release(d, (int)out[j] - 1);
            return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
        }
        d->surfaces[si].kind = MTKVCP_SURF_UNBOUND;
        d->surfaces[si].state = MTKVCP_SS_IDLE;
        d->surfaces[si].width = w;
        d->surfaces[si].height = h;
        d->surfaces[si].ctx = -1;
        d->surfaces[si].cap_index = -1;
        d->surfaces[si].prime_fd = -1;
        d->surfaces[si].bounce_fd = -1;
        d->surfaces[si].imp_fd = -1;
        d->surfaces[si].imp_map = NULL;
        d->surfaces[si].alias_fd = -1;
        d->surfaces[si].fourcc = rt_format == VA_RT_FORMAT_YUV420_10 ?
                                V4L2_PIX_FMT_P010 : V4L2_PIX_FMT_NV12;
        d->surfaces[si].enc_data = NULL;
        d->surfaces[si].enc_rgb = -1;
        out[i] = (VASurfaceID)(si + 1);
    }
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_CreateSurfaces(VADriverContextP ctx, int w, int h,
    int format, int n, VASurfaceID *surfaces)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    VAStatus st;
    if (!surfaces || n <= 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    pthread_mutex_lock(&d->lock);
    st = mtkvcp_create_surfs_locked(d, w, h, (unsigned int)format, n,
                                    surfaces);
    pthread_mutex_unlock(&d->lock);
    return st;
}

static VAStatus mtkvcp_DestroySurfaces(VADriverContextP ctx,
    VASurfaceID *list, int n)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int i;
    if (!list || n <= 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    pthread_mutex_lock(&d->lock);
    for (i = 0; i < n; i++) {
        int si = (int)list[i] - 1;
        if (si < 0 || si >= MTKVCP_MAX_SURFACES ||
            !d->surfaces[si].in_use) {
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_ERROR_INVALID_SURFACE;
        }
        if (d->surfaces[si].ctx >= 0) {
            int ci = d->surfaces[si].ctx;
            if (ci >= 0 && ci < MTKVCP_MAX_CONTEXTS &&
                d->contexts[ci].in_use && d->contexts[ci].streaming) {
                pthread_mutex_unlock(&d->lock);
                return VA_STATUS_ERROR_SURFACE_BUSY;
            }
        }
    }
    for (i = 0; i < n; i++) {
        int si = (int)list[i] - 1;
        /* Imported dma-bufs are shared: several VA surfaces handed to
         * the client may alias one slot, so destroy only drops a
         * reference until the last handle goes away. */
        if (d->surfaces[si].kind == MTKVCP_SURF_PRIME_IMPORT &&
            d->surfaces[si].imp_refs > 1) {
            d->surfaces[si].imp_refs--;
            continue;
        }
        mtkvcp_surface_release(d, si);
    }
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_CreateSurfaces2(VADriverContextP ctx,
    unsigned int format, unsigned int w, unsigned int h,
    VASurfaceID *surfaces, unsigned int n,
    VASurfaceAttrib *attribs, unsigned int num_attribs)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    unsigned int i;
    unsigned int rt = 0;
    int mem_type = 0, have_mem_type = 0;
    const VADRMPRIMESurfaceDescriptor *import = NULL;
    const VASurfaceAttribExternalBuffers *import_v1 = NULL;
    VAStatus st;
    if (!surfaces || !n)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    for (i = 0; i < num_attribs; i++) {
        if (attribs[i].type == VASurfaceAttribMemoryType) {
            have_mem_type = 1;
            mem_type = attribs[i].value.value.i;
        } else if (attribs[i].type ==
                   VASurfaceAttribExternalBufferDescriptor) {
            /* The descriptor's meaning depends on the memory type, and
             * the two flavours are unrelated structures: PRIME_2 uses
             * VADRMPRIMESurfaceDescriptor, the legacy PRIME type uses
             * VASurfaceAttribExternalBuffers. */
            if (mem_type == VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME)
                import_v1 = (const VASurfaceAttribExternalBuffers *)
                            attribs[i].value.value.p;
            else
                import = (const VADRMPRIMESurfaceDescriptor *)
                         attribs[i].value.value.p;
        }
    }
    if (import_v1) {
        if (n != 1)
            return VA_STATUS_ERROR_INVALID_PARAMETER;
        pthread_mutex_lock(&d->lock);
        st = mtkvcp_vpp_import_surface_v1(d, (int)w, (int)h, import_v1,
                                          surfaces);
        pthread_mutex_unlock(&d->lock);
        return st;
    }
    if (import || (have_mem_type &&
                   mem_type != VA_SURFACE_ATTRIB_MEM_TYPE_VA)) {
        /* DMA-BUF import (hwmap=mode=direct style): one descriptor
         * maps to exactly one surface. */
        mtkvcp_log("CreateSurfaces2 import path: import=%p mem_type=0x%x "
                   "have_mem_type=%d n=%u", (const void *)import, mem_type,
                   have_mem_type, n);
        if (!import || n != 1)
            return VA_STATUS_ERROR_INVALID_PARAMETER;
        if (have_mem_type &&
            mem_type != VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2)
            return VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE;
        pthread_mutex_lock(&d->lock);
        st = mtkvcp_vpp_import_surface(d, (int)w, (int)h, import,
                                       surfaces);
        pthread_mutex_unlock(&d->lock);
        return st;
    }
    if (format != VA_RT_FORMAT_YUV420 && format != VA_RT_FORMAT_YUV420_10)
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    for (i = 0; i < num_attribs; i++) {
        if (attribs[i].type == VASurfaceAttribPixelFormat) {
            if (attribs[i].value.value.i != (int)VA_FOURCC_NV12 &&
                attribs[i].value.value.i != (int)VA_FOURCC_P010)
                return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
            rt = (attribs[i].value.value.i == (int)VA_FOURCC_P010) ?
                 VA_RT_FORMAT_YUV420_10 : VA_RT_FORMAT_YUV420;
        }
    }
    if (!rt)
        rt = format;
    pthread_mutex_lock(&d->lock);
    st = mtkvcp_create_surfs_locked(d, (int)w, (int)h, rt, (int)n,
                                    surfaces);
    pthread_mutex_unlock(&d->lock);
    return st;
}

static VAStatus mtkvcp_QuerySurfaceAttributes(VADriverContextP ctx,
    VAConfigID config, VASurfaceAttrib *attribs, unsigned int *num)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    const struct mtkvcp_prof *p;
    int ci = (int)config - 1;
    int profile, entrypoint;
    if (!num)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (ci < 0 || ci >= MTKVCP_MAX_CONTEXTS)
        return VA_STATUS_ERROR_INVALID_CONFIG;
    pthread_mutex_lock(&d->lock);
    if (!d->configs[ci].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_CONFIG;
    }
    profile = d->configs[ci].profile;
    entrypoint = d->configs[ci].entrypoint;
    pthread_mutex_unlock(&d->lock);
    p = mtkvcp_find_prof(profile, entrypoint);
    if (!p)
        return VA_STATUS_ERROR_INVALID_CONFIG;
    if (!attribs) {
        *num = 5;
        return VA_STATUS_SUCCESS;
    }
    if (*num < 5) {
        *num = 5;
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }
    attribs[0].type = VASurfaceAttribPixelFormat;
    attribs[0].flags = VA_SURFACE_ATTRIB_GETTABLE;
    attribs[0].value.type = VAGenericValueTypeInteger;
    attribs[0].value.value.i = p->is_10bit ? (int)VA_FOURCC_P010 :
                               (int)VA_FOURCC_NV12;
    attribs[1].type = VASurfaceAttribMinWidth;
    attribs[1].flags = VA_SURFACE_ATTRIB_GETTABLE;
    attribs[1].value.type = VAGenericValueTypeInteger;
    attribs[1].value.value.i = MTKVCP_MIN_W;
    attribs[2].type = VASurfaceAttribMinHeight;
    attribs[2].flags = VA_SURFACE_ATTRIB_GETTABLE;
    attribs[2].value.type = VAGenericValueTypeInteger;
    attribs[2].value.value.i = MTKVCP_MIN_H;
    attribs[3].type = VASurfaceAttribMaxWidth;
    attribs[3].flags = VA_SURFACE_ATTRIB_GETTABLE;
    attribs[3].value.type = VAGenericValueTypeInteger;
    attribs[3].value.value.i = MTKVCP_MAX_W;
    attribs[4].type = VASurfaceAttribMaxHeight;
    attribs[4].flags = VA_SURFACE_ATTRIB_GETTABLE;
    attribs[4].value.type = VAGenericValueTypeInteger;
    attribs[4].value.value.i = MTKVCP_MAX_H;
    *num = 5;
    return VA_STATUS_SUCCESS;
}

/* ---- contexts ---- */
static VAStatus mtkvcp_CreateContext(VADriverContextP ctx, VAConfigID cfg,
    int w, int h, int flag, VASurfaceID *targets, int num_targets,
    VAContextID *context)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    const struct mtkvcp_prof *p;
    int ci, cci, i;
    VAStatus st;
    (void)flag;
    if (!context)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    cci = (int)cfg - 1;
    if (cci < 0 || cci >= MTKVCP_MAX_CONTEXTS)
        return VA_STATUS_ERROR_INVALID_CONFIG;
    pthread_mutex_lock(&d->lock);
    if (!d->configs[cci].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_CONFIG;
    }
    p = mtkvcp_find_prof(d->configs[cci].profile,
                         d->configs[cci].entrypoint);
    if (!p) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_CONFIG;
    }
    if (w < MTKVCP_MIN_W || h < MTKVCP_MIN_H ||
        w > MTKVCP_MAX_W || h > MTKVCP_MAX_H) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED;
    }
    ci = mtkvcp_alloc_context(d);
    if (ci < 0) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }
    memset(&d->contexts[ci], 0, sizeof(d->contexts[ci]));
    d->contexts[ci].in_use = 1;
    d->contexts[ci].replay_idr = -1;
    d->contexts[ci].is_vpp = p->is_vpp;
    d->contexts[ci].config = cci;
    d->contexts[ci].is_encode = p->is_encode;
    d->contexts[ci].vfd = -1;
    d->contexts[ci].width = w;
    d->contexts[ci].height = h;
    d->contexts[ci].out_fourcc = p->out_fourcc;
    d->contexts[ci].cap_fourcc = p->cap_fourcc;
    /* Both the encoder and the post-processing step participate in the
     * packed-RGB capture path; see mtkvcp_rgb_input_mode. */
    d->contexts[ci].rgb_in = mtkvcp_rgb_input_mode();
    d->contexts[ci].au_target = -1;
    d->contexts[ci].enc_target = -1;
    d->contexts[ci].enc_coded_buf = -1;
    d->contexts[ci].pend_head = d->contexts[ci].pend_tail = 0;
    d->contexts[ci].vpp_target = -1;
    if (p->is_vpp) {
        /* CPU-only: no V4L2 session, no render-target binding.
         * Tolerate (but ignore) client-supplied targets. */
        for (i = 0; i < num_targets && targets; i++) {
            int si = (int)targets[i] - 1;
            if (si < 0 || si >= MTKVCP_MAX_SURFACES ||
                !d->surfaces[si].in_use ||
                d->surfaces[si].width != w ||
                d->surfaces[si].height != h) {
                d->contexts[ci].in_use = 0;
                pthread_mutex_unlock(&d->lock);
                return VA_STATUS_ERROR_INVALID_SURFACE;
            }
        }
        pthread_mutex_unlock(&d->lock);
        *context = (VAContextID)(ci + 1);
        mtkvcp_log("CreateContext vpp %dx%d -> %d", w, h, ci + 1);
        return VA_STATUS_SUCCESS;
    }
    /* Pre-bind the render targets the client hands us. */
    for (i = 0; i < num_targets && targets; i++) {
        int si = (int)targets[i] - 1;
        if (si < 0 || si >= MTKVCP_MAX_SURFACES ||
            !d->surfaces[si].in_use ||
            d->surfaces[si].ctx >= 0 ||
            d->surfaces[si].width != w ||
            d->surfaces[si].height != h) {
            d->contexts[ci].in_use = 0;
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_ERROR_INVALID_SURFACE;
        }
        d->surfaces[si].ctx = ci;
        d->surfaces[si].kind = p->is_encode ? MTKVCP_SURF_ENC_INPUT :
                               MTKVCP_SURF_DECODE;
    }
    st = p->is_encode ? mtkvcp_enc_create(d, ci) :
                        mtkvcp_dec_create(d, ci);
    if (st != VA_STATUS_SUCCESS) {
        for (i = 0; i < num_targets && targets; i++) {
            int si = (int)targets[i] - 1;
            if (si >= 0 && si < MTKVCP_MAX_SURFACES) {
                d->surfaces[si].ctx = -1;
                d->surfaces[si].kind = MTKVCP_SURF_UNBOUND;
            }
        }
        d->contexts[ci].in_use = 0;
        pthread_mutex_unlock(&d->lock);
        return st;
    }
    pthread_mutex_unlock(&d->lock);
    *context = (VAContextID)(ci + 1);
    mtkvcp_log("CreateContext %s %dx%d -> %d (%d targets)",
               p->is_encode ? "enc" : "dec", w, h, ci + 1, num_targets);
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_DestroyContext(VADriverContextP ctx, VAContextID id)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int ci = (int)id - 1;
    int i;
    if (ci < 0 || ci >= MTKVCP_MAX_CONTEXTS)
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    pthread_mutex_lock(&d->lock);
    if (!d->contexts[ci].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    }
    if (d->contexts[ci].is_encode)
        mtkvcp_enc_destroy(d, ci);
    else if (!d->contexts[ci].is_vpp)
        mtkvcp_dec_destroy(d, ci);
    /* VPP contexts hold no V4L2 session. */
    /* Detach (do not free) bound surfaces; the client destroys them. */
    for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
        if (d->surfaces[i].in_use && d->surfaces[i].ctx == ci) {
            if (d->surfaces[i].prime_fd >= 0) {
                close(d->surfaces[i].prime_fd);
                d->surfaces[i].prime_fd = -1;
            }
            if (d->surfaces[i].kind == MTKVCP_SURF_DECODE_STAGED) {
                /* Keep the staged frame. VAAPI keeps a surface valid
                 * after its context is destroyed, so a decoded frame the
                 * client has not read yet must stay readable; the
                 * UNBOUND-with-enc_data path serves it from CPU memory. */
                d->surfaces[i].ctx = -1;
                d->surfaces[i].kind = MTKVCP_SURF_UNBOUND;
                d->surfaces[i].state = MTKVCP_SS_IDLE;
                d->surfaces[i].cap_index = -1;
                continue;
            }
            free(d->surfaces[i].enc_data);
            d->surfaces[i].enc_data = NULL;
            d->surfaces[i].ctx = -1;
            d->surfaces[i].kind = MTKVCP_SURF_UNBOUND;
            d->surfaces[i].state = MTKVCP_SS_IDLE;
            d->surfaces[i].cap_index = -1;
        }
    free(d->contexts[ci].au);
    free(d->contexts[ci].enc_au);
    {
        int ri;

        for (ri = 0; ri < MTKVCP_REPLAY_MAX; ri++)
            free(d->contexts[ci].replay_au[ri]);
    }
    d->contexts[ci].in_use = 0;
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

/* ---- buffers ---- */
static VAStatus mtkvcp_CreateBuffer(VADriverContextP ctx, VAContextID c,
    VABufferType type, unsigned int size, unsigned int num_elements,
    void *data, VABufferID *buf_id)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int bi;
    size_t total;
    (void)c;
    if (!buf_id || size == 0 || num_elements == 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    total = (size_t)size * (size_t)num_elements;
    if (total / size != num_elements || total > (64u * 1024u * 1024u))
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    switch (type) {
    case VAPictureParameterBufferType:
    case VAIQMatrixBufferType:
    case VABitPlaneBufferType:
    case VASliceGroupMapBufferType:
    case VASliceParameterBufferType:
    case VASliceDataBufferType:
    case VAMacroblockParameterBufferType:
    case VAResidualDataBufferType:
    case VADeblockingParameterBufferType:
    case VAImageBufferType:
    case VAProtectedSliceDataBufferType:
    case VAEncCodedBufferType:
    case VAEncSequenceParameterBufferType:
    case VAEncPictureParameterBufferType:
    case VAEncSliceParameterBufferType:
    case VAEncPackedHeaderParameterBufferType:
    case VAEncPackedHeaderDataBufferType:
    case VAEncMiscParameterBufferType:
    case VAEncMacroblockParameterBufferType:
    case VAEncMacroblockMapBufferType:
    case VAProcPipelineParameterBufferType:
        break;
    default:
        return VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE;
    }
    pthread_mutex_lock(&d->lock);
    bi = mtkvcp_alloc_buffer(d);
    if (bi < 0) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }
    d->buffers[bi].ctx = (int)c - 1;
    d->buffers[bi].type = type;
    d->buffers[bi].size = size;
    d->buffers[bi].num_elements = num_elements;
    /* A coded buffer's size is payload capacity; its VA mapping is the
     * segment metadata chain (F8/F14). */
    d->buffers[bi].data = type == VAEncCodedBufferType ?
        calloc(num_elements, sizeof(VACodedBufferSegment)) :
        malloc(total ? total : 1);
    d->buffers[bi].is_coded_seg = 0;
    d->buffers[bi].coded_bytes = NULL;
    d->buffers[bi].coded_size = 0;
    d->buffers[bi].handle_fd = -1;
    d->buffers[bi].handle_size = 0;
    if (!d->buffers[bi].data) {
        d->buffers[bi].in_use = 0;
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    if (type == VAEncCodedBufferType) {
        /* calloc above already zeroed the segment metadata. */
    } else if (data) {
        memcpy(d->buffers[bi].data, data, total);
    } else {
        memset(d->buffers[bi].data, 0, total);
    }
    pthread_mutex_unlock(&d->lock);
    *buf_id = (VABufferID)(bi + 1);
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_BufferSetNumElements(VADriverContextP ctx,
    VABufferID id, unsigned int n)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int bi = (int)id - 1;
    void *nd;
    size_t total;
    VAStatus st;
    if (bi < 0 || bi >= MTKVCP_MAX_BUFFERS)
        return VA_STATUS_ERROR_INVALID_BUFFER;
    pthread_mutex_lock(&d->lock);
    if (!d->buffers[bi].in_use || n == 0) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
    if (d->buffers[bi].coded_busy) {
        st = mtkvcp_enc_wait_buffer(d, bi, 15ull * 1000000000ull);
        if (st != VA_STATUS_SUCCESS || d->buffers[bi].coded_busy) {
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_ERROR_SURFACE_BUSY;
        }
    }
    total = d->buffers[bi].type == VAEncCodedBufferType ?
        (size_t)sizeof(VACodedBufferSegment) * n :
        (size_t)d->buffers[bi].size * n;
    if (d->buffers[bi].type != VAEncCodedBufferType &&
        total / d->buffers[bi].size != n) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_VALUE;
    }
    nd = realloc(d->buffers[bi].data, total ? total : 1);
    if (!nd) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    d->buffers[bi].data = nd;
    if (d->buffers[bi].is_coded_seg) {
        size_t payload = (size_t)d->buffers[bi].size * n;

        nd = realloc(d->buffers[bi].coded_bytes, payload ? payload : 1);
        if (!nd) {
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        d->buffers[bi].coded_bytes = nd;
        d->buffers[bi].coded_capacity = payload;
    }
    d->buffers[bi].num_elements = n;
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_MapBuffer(VADriverContextP ctx, VABufferID id,
    void **pbuf)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int bi = (int)id - 1;
    if (!pbuf)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (bi < 0 || bi >= MTKVCP_MAX_BUFFERS)
        return VA_STATUS_ERROR_INVALID_BUFFER;
    pthread_mutex_lock(&d->lock);
    if (!d->buffers[bi].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
    if (d->buffers[bi].is_coded_seg) {
        VAStatus st = mtkvcp_enc_wait_buffer(d, bi, (uint64_t)-1);

        if (st != VA_STATUS_SUCCESS) {
            pthread_mutex_unlock(&d->lock);
            return st;
        }
    }
    *pbuf = d->buffers[bi].is_coded_seg ? d->buffers[bi].data :
              d->buffers[bi].map_alias ? d->buffers[bi].map_alias :
                                         d->buffers[bi].data;
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_MapBuffer2(VADriverContextP ctx, VABufferID id,
    void **pbuf, uint32_t flags)
{
    (void)flags;
    return mtkvcp_MapBuffer(ctx, id, pbuf);
}

static VAStatus mtkvcp_UnmapBuffer(VADriverContextP ctx, VABufferID id)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int bi = (int)id - 1;
    if (bi < 0 || bi >= MTKVCP_MAX_BUFFERS)
        return VA_STATUS_ERROR_INVALID_BUFFER;
    pthread_mutex_lock(&d->lock);
    if (!d->buffers[bi].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_DestroyBuffer(VADriverContextP ctx, VABufferID id)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int bi = (int)id - 1;
    if (bi < 0 || bi >= MTKVCP_MAX_BUFFERS)
        return VA_STATUS_ERROR_INVALID_BUFFER;
    pthread_mutex_lock(&d->lock);
    if (!d->buffers[bi].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
    if (d->buffers[bi].coded_busy) {
        VAStatus st = mtkvcp_enc_wait_buffer(d, bi, 15ull * 1000000000ull);

        if (st != VA_STATUS_SUCCESS || d->buffers[bi].coded_busy) {
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_ERROR_SURFACE_BUSY;
        }
    }
    if (d->buffers[bi].handle_fd >= 0)
        close(d->buffers[bi].handle_fd);
    free(d->buffers[bi].data);
    free(d->buffers[bi].coded_bytes);
    memset(&d->buffers[bi], 0, sizeof(d->buffers[bi]));
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_BufferInfo(VADriverContextP ctx, VABufferID id,
    VABufferType *type, unsigned int *size, unsigned int *num_elements)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int bi = (int)id - 1;
    if (bi < 0 || bi >= MTKVCP_MAX_BUFFERS)
        return VA_STATUS_ERROR_INVALID_BUFFER;
    pthread_mutex_lock(&d->lock);
    if (!d->buffers[bi].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
    if (type)
        *type = d->buffers[bi].type;
    if (size)
        *size = d->buffers[bi].size;
    if (num_elements)
        *num_elements = d->buffers[bi].num_elements;
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

/* ---- picture dispatch ---- */
static VAStatus mtkvcp_BeginPicture(VADriverContextP ctx, VAContextID c,
    VASurfaceID target)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int ci = (int)c - 1, si = (int)target - 1;
    VAStatus st;
    if (ci < 0 || ci >= MTKVCP_MAX_CONTEXTS)
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    if (si < 0 || si >= MTKVCP_MAX_SURFACES)
        return VA_STATUS_ERROR_INVALID_SURFACE;
    pthread_mutex_lock(&d->lock);
    if (!d->contexts[ci].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    }
    if (!d->surfaces[si].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    if (d->contexts[ci].error) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_DECODING_ERROR;
    }
    st = d->contexts[ci].is_vpp ? mtkvcp_vpp_begin(d, ci, si) :
         d->contexts[ci].is_encode ? mtkvcp_enc_begin(d, ci, si) :
                                     mtkvcp_dec_begin(d, ci, si);
    pthread_mutex_unlock(&d->lock);
    return st;
}

static VAStatus mtkvcp_RenderPicture(VADriverContextP ctx, VAContextID c,
    VABufferID *buffers, int num_buffers)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int ci = (int)c - 1, i;
    VAStatus st = VA_STATUS_SUCCESS;
    if (ci < 0 || ci >= MTKVCP_MAX_CONTEXTS)
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    if (!buffers || num_buffers <= 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    pthread_mutex_lock(&d->lock);
    if (!d->contexts[ci].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    }
    mtkvcp_log("RenderPicture ctx=%d n=%d", ci + 1, num_buffers);
    for (i = 0; i < num_buffers; i++) {
        int bi = (int)buffers[i] - 1;
        if (bi < 0 || bi >= MTKVCP_MAX_BUFFERS ||
            !d->buffers[bi].in_use) {
            st = VA_STATUS_ERROR_INVALID_BUFFER;
            break;
        }
        if (getenv("MTK_VCP_VA_TRACE_VERBOSE"))
        mtkvcp_log("  render buf=%d type=%u size=%u num=%u", buffers[i],
                   d->buffers[bi].type, d->buffers[bi].size,
                   d->buffers[bi].num_elements);
        st = d->contexts[ci].is_vpp ? mtkvcp_vpp_render(d, ci, bi) :
             d->contexts[ci].is_encode ? mtkvcp_enc_render(d, ci, bi) :
                                         mtkvcp_dec_render(d, ci, bi);
        if (st != VA_STATUS_SUCCESS)
            break;
    }
    pthread_mutex_unlock(&d->lock);
    return st;
}

static VAStatus mtkvcp_EndPicture(VADriverContextP ctx, VAContextID c)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int ci = (int)c - 1;
    VAStatus st;
    if (ci < 0 || ci >= MTKVCP_MAX_CONTEXTS)
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    pthread_mutex_lock(&d->lock);
    if (!d->contexts[ci].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    }
    st = d->contexts[ci].is_vpp ? mtkvcp_vpp_end(d, ci) :
         d->contexts[ci].is_encode ? mtkvcp_enc_end(d, ci) :
                                     mtkvcp_dec_end(d, ci);
    pthread_mutex_unlock(&d->lock);
    if (st != VA_STATUS_SUCCESS)
        mtkvcp_log("EndPicture ctx=%d -> %s", ci + 1,
                   mtkvcp_va_status_str(st));
    return st;
}

static VAStatus mtkvcp_SyncSurface(VADriverContextP ctx, VASurfaceID s)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int si = (int)s - 1;
    VAStatus st;
    if (si < 0 || si >= MTKVCP_MAX_SURFACES)
        return VA_STATUS_ERROR_INVALID_SURFACE;
    pthread_mutex_lock(&d->lock);
    if (!d->surfaces[si].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    if (mtkvcp_enc_surface_busy(d, si)) {
        st = mtkvcp_enc_sync_surface(d, si, (uint64_t)-1);
        pthread_mutex_unlock(&d->lock);
        return st;
    }
    /* Only a TARGET surface has in-flight work; anything else is
     * trivially complete (upload staging, idle slots). */
    if (d->surfaces[si].kind != MTKVCP_SURF_DECODE ||
        d->surfaces[si].state != MTKVCP_SS_TARGET) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_SUCCESS;
    }
    st = mtkvcp_dec_sync(d, si, 0);
    pthread_mutex_unlock(&d->lock);
    return st;
}

static VAStatus mtkvcp_SyncSurface2(VADriverContextP ctx, VASurfaceID s,
    uint64_t timeout_ns)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int si = (int)s - 1;
    VAStatus st;
    if (si < 0 || si >= MTKVCP_MAX_SURFACES)
        return VA_STATUS_ERROR_INVALID_SURFACE;
    pthread_mutex_lock(&d->lock);
    if (!d->surfaces[si].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    if (mtkvcp_enc_surface_busy(d, si)) {
        st = mtkvcp_enc_sync_surface(d, si, timeout_ns);
        pthread_mutex_unlock(&d->lock);
        return st;
    }
    if (d->surfaces[si].kind != MTKVCP_SURF_DECODE ||
        d->surfaces[si].state != MTKVCP_SS_TARGET) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_SUCCESS;
    }
    st = mtkvcp_dec_sync(d, si, timeout_ns);
    pthread_mutex_unlock(&d->lock);
    return st;
}

static VAStatus mtkvcp_QuerySurfaceStatus(VADriverContextP ctx,
    VASurfaceID s, VASurfaceStatus *status)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int si = (int)s - 1;
    if (!status)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (si < 0 || si >= MTKVCP_MAX_SURFACES)
        return VA_STATUS_ERROR_INVALID_SURFACE;
    pthread_mutex_lock(&d->lock);
    if (!d->surfaces[si].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    *status = (d->surfaces[si].state == MTKVCP_SS_TARGET ||
               mtkvcp_enc_surface_busy(d, si)) ?
              VASurfaceRendering : VASurfaceReady;
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_PutSurface(VADriverContextP ctx, VASurfaceID s,
    void *draw, short srcx, short srcy, unsigned short srcw,
    unsigned short srch, short destx, short desty, unsigned short destw,
    unsigned short desth, VARectangle *clips, unsigned int nclips,
    unsigned int flags)
{
    (void)ctx; (void)s; (void)draw; (void)srcx; (void)srcy; (void)srcw;
    (void)srch; (void)destx; (void)desty; (void)destw; (void)desth;
    (void)clips; (void)nclips; (void)flags;
    /* No overlay path: Wayland clients use PRIME export. */
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

static VAStatus mtkvcp_SyncBuffer(VADriverContextP ctx, VABufferID id,
    uint64_t timeout_ns)
{
    struct mtkvcp_drv *d = mtkvcp_drv_of(ctx);
    int bi = (int)id - 1;
    VAStatus st;
    if (bi < 0 || bi >= MTKVCP_MAX_BUFFERS)
        return VA_STATUS_ERROR_INVALID_BUFFER;
    pthread_mutex_lock(&d->lock);
    if (!d->buffers[bi].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
    st = mtkvcp_enc_wait_buffer(d, bi, timeout_ns);
    pthread_mutex_unlock(&d->lock);
    return st;
}

static VAStatus mtkvcp_QueryDisplayAttributes(VADriverContextP ctx,
    VADisplayAttribute *attrs, int *num)
{
    (void)ctx; (void)attrs;
    if (!num)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    *num = 0;
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_GetDisplayAttributes(VADriverContextP ctx,
    VADisplayAttribute *attrs, int num)
{
    (void)ctx; (void)attrs; (void)num;
    return VA_STATUS_SUCCESS;
}

static VAStatus mtkvcp_SetDisplayAttributes(VADriverContextP ctx,
    VADisplayAttribute *attrs, int num)
{
    (void)ctx; (void)attrs; (void)num;
    return VA_STATUS_ERROR_ATTR_NOT_SUPPORTED;
}

/* ---- driver entry ---- */
VAStatus __vaDriverInit_1_0(VADriverContextP ctx)
{
    struct VADriverVTable *vt;
    struct mtkvcp_drv *d;
    if (!ctx)
        return VA_STATUS_ERROR_INVALID_DISPLAY;
    d = calloc(1, sizeof(*d));
    if (!d)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    pthread_mutex_init(&d->lock, NULL);
    d->hw_owner_ctx = -1;
    vt = ctx->vtable;
    vt->vaTerminate = mtkvcp_Terminate;
    vt->vaQueryConfigProfiles = mtkvcp_QueryConfigProfiles;
    vt->vaQueryConfigEntrypoints = mtkvcp_QueryConfigEntrypoints;
    vt->vaGetConfigAttributes = mtkvcp_GetConfigAttributes;
    vt->vaCreateConfig = mtkvcp_CreateConfig;
    vt->vaDestroyConfig = mtkvcp_DestroyConfig;
    vt->vaQueryConfigAttributes = mtkvcp_QueryConfigAttributes;
    vt->vaCreateSurfaces = mtkvcp_CreateSurfaces;
    vt->vaDestroySurfaces = mtkvcp_DestroySurfaces;
    vt->vaCreateContext = mtkvcp_CreateContext;
    vt->vaDestroyContext = mtkvcp_DestroyContext;
    vt->vaCreateBuffer = mtkvcp_CreateBuffer;
    vt->vaBufferSetNumElements = mtkvcp_BufferSetNumElements;
    vt->vaMapBuffer = mtkvcp_MapBuffer;
    vt->vaUnmapBuffer = mtkvcp_UnmapBuffer;
    vt->vaDestroyBuffer = mtkvcp_DestroyBuffer;
    vt->vaBeginPicture = mtkvcp_BeginPicture;
    vt->vaRenderPicture = mtkvcp_RenderPicture;
    vt->vaEndPicture = mtkvcp_EndPicture;
    vt->vaSyncSurface = mtkvcp_SyncSurface;
    vt->vaQuerySurfaceStatus = mtkvcp_QuerySurfaceStatus;
    vt->vaQuerySurfaceError = (void *)mtkvcp_not_supported;
    vt->vaPutSurface = mtkvcp_PutSurface;
    vt->vaQueryImageFormats = mtkvcp_QueryImageFormats;
    vt->vaCreateImage = mtkvcp_CreateImage;
    vt->vaDeriveImage = mtkvcp_DeriveImage;
    vt->vaDestroyImage = mtkvcp_DestroyImage;
    vt->vaSetImagePalette = (void *)mtkvcp_not_supported;
    vt->vaGetImage = mtkvcp_GetImage;
    vt->vaPutImage = mtkvcp_PutImage;
    vt->vaQuerySubpictureFormats = (void *)mtkvcp_not_supported;
    vt->vaCreateSubpicture = (void *)mtkvcp_not_supported;
    vt->vaDestroySubpicture = (void *)mtkvcp_not_supported;
    vt->vaSetSubpictureImage = (void *)mtkvcp_not_supported;
    vt->vaSetSubpictureChromakey = (void *)mtkvcp_not_supported;
    vt->vaSetSubpictureGlobalAlpha = (void *)mtkvcp_not_supported;
    vt->vaAssociateSubpicture = (void *)mtkvcp_not_supported;
    vt->vaDeassociateSubpicture = (void *)mtkvcp_not_supported;
    vt->vaQueryDisplayAttributes = mtkvcp_QueryDisplayAttributes;
    vt->vaGetDisplayAttributes = mtkvcp_GetDisplayAttributes;
    vt->vaSetDisplayAttributes = mtkvcp_SetDisplayAttributes;
    vt->vaBufferInfo = mtkvcp_BufferInfo;
    vt->vaLockSurface = (void *)mtkvcp_not_supported;
    vt->vaUnlockSurface = (void *)mtkvcp_not_supported;
    vt->vaGetSurfaceAttributes = (void *)mtkvcp_not_supported;
    vt->vaCreateSurfaces2 = mtkvcp_CreateSurfaces2;
    vt->vaQuerySurfaceAttributes = mtkvcp_QuerySurfaceAttributes;
    vt->vaAcquireBufferHandle = mtkvcp_AcquireBufferHandle;
    vt->vaReleaseBufferHandle = mtkvcp_ReleaseBufferHandle;
    vt->vaCreateMFContext = (void *)mtkvcp_not_supported;
    vt->vaMFAddContext = (void *)mtkvcp_not_supported;
    vt->vaMFReleaseContext = (void *)mtkvcp_not_supported;
    vt->vaMFSubmit = (void *)mtkvcp_not_supported;
    vt->vaCreateBuffer2 = (void *)mtkvcp_not_supported;
    vt->vaQueryProcessingRate = (void *)mtkvcp_not_supported;
    vt->vaExportSurfaceHandle = mtkvcp_ExportSurfaceHandle;
    vt->vaSyncSurface2 = mtkvcp_SyncSurface2;
    vt->vaSyncBuffer = mtkvcp_SyncBuffer;
    vt->vaCopy = (void *)mtkvcp_not_supported;
    vt->vaMapBuffer2 = mtkvcp_MapBuffer2;
    ctx->version_major = VA_MAJOR_VERSION;
    ctx->version_minor = VA_MINOR_VERSION;
    ctx->max_profiles = 12;
    ctx->max_entrypoints = 3;
    ctx->max_attributes = 16;
    ctx->max_image_formats = 4;
    ctx->max_subpic_formats = 1;
    ctx->max_display_attributes = 1;
    ctx->str_vendor = "MTK VCP VAAPI backend (stateful V4L2, single session)";
    ctx->pDriverData = d;
    {
        /* Separate VPP vtable (libva >= 1.x video post-processing). */
        struct VADriverVTableVPP *vpp =
            calloc(1, sizeof(*vpp));
        if (!vpp) {
            pthread_mutex_destroy(&d->lock);
            free(d);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        vpp->version = VA_DRIVER_VTABLE_VPP_VERSION;
        vpp->vaQueryVideoProcFilters = mtkvcp_vpp_query_filters;
        vpp->vaQueryVideoProcFilterCaps = mtkvcp_vpp_query_filter_caps;
        vpp->vaQueryVideoProcPipelineCaps =
            mtkvcp_vpp_query_pipeline_caps;
        ctx->vtable_vpp = vpp;
    }
    mtkvcp_log("driver init");
    return VA_STATUS_SUCCESS;
}
