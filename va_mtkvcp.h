/* SPDX-License-Identifier: MIT */
/*
 * VAAPI backend for the MediaTek MT6895 VCP stateful V4L2 M2M codec.
 *
 * Maps VA-API decode (VLD) and slice-level encode (EncSlice) onto
 * /dev/video1 (mtk-vcp-dec, stateful decoder) and /dev/video0
 * (mtk-vcodec-enc, stateful encoder).
 *
 * Key design points (all driven by verified device behaviour):
 * - The V4L2 driver is STATEFUL: it parses the coded stream itself.
 *   Per-picture VA slice data is concatenated into one access unit and
 *   submitted with a single OUTPUT QBUF, exactly like the validated
 *   V4L2 probes. Slice/picture parameter contents are informational.
 * - The VCP holds a single global session shared by encode and decode.
 *   The backend claims a process-wide owner at first EndPicture and
 *   releases it at context destroy. A second streaming context fails
 *   with VA_STATUS_ERROR_HW_BUSY; cross-process contention surfaces
 *   as -EBUSY from STREAMON and is mapped the same way.
 * - CAPTURE buffers are the VA surfaces. The firmware fills them in
 *   FIFO order, which may differ from the client's render_target, so
 *   completion swaps the underlying buffer index between the target
 *   surface and the surface previously owning the filled buffer.
 * - No mid-stream resolution change in v1: a SOURCE_CHANGE event puts
 *   the context into error and the client must re-create it.
 */
#ifndef VA_MTKVCP_H
#define VA_MTKVCP_H

#include <stdint.h>
#include <pthread.h>
#include <linux/videodev2.h>
#include <va/va.h>
#include <va/va_dec_hevc.h>
#include <va/va_vpp.h>
#include <va/va_drmcommon.h>

/* ---- driver nodes (overridable for tests) ---- */
#define MTKVCP_DEC_NODE "/dev/video1"
#define MTKVCP_ENC_NODE "/dev/video0"

/* ---- pool sizing ---- */
#define MTKVCP_MAX_CONTEXTS   16
#define MTKVCP_MAX_SURFACES   64
#define MTKVCP_MAX_BUFFERS    256
#define MTKVCP_MAX_IMAGES     64
/* AUs buffered per decode context so a mid-stream drain can rebuild the
 * firmware DPB (see the replay fields in struct mtkvcp_context). */
#define MTKVCP_REPLAY_MAX     64
#define MTKVCP_OUT_BUFS_DEC   8   /* OUTPUT (coded in) ring */
#define MTKVCP_OUT_BUFS_ENC   4
#define MTKVCP_CAP_BUFS_ENC   6   /* coded-out ring for encode */
#define MTKVCP_ENC_MAX_JOBS   8   /* jobs may outlive their OUTPUT slot */
#define MTKVCP_OUT_SIZE_MAX   (4u * 1024u * 1024u) /* one AU must fit */

/* Verified envelope (see docs/video/CAPABILITIES.md). */
#define MTKVCP_MAX_W 3840
#define MTKVCP_MAX_H 2160
#define MTKVCP_MIN_W 64
#define MTKVCP_MIN_H 64

/* Surface kinds. UNBOUND surfaces are bound on first vaBeginPicture. */
enum mtkvcp_surf_kind {
    MTKVCP_SURF_UNBOUND = 0,
    MTKVCP_SURF_DECODE,     /* wraps one V4L2 CAPTURE buffer */
    MTKVCP_SURF_ENC_INPUT,  /* CPU-side NV12 image feeding the encoder */
    MTKVCP_SURF_PRIME_IMPORT, /* wraps an imported dma-buf (VPP input) */
    /* A decoded frame staged to CPU memory because its context was torn
     * down before the client read it. VAAPI keeps surfaces valid past
     * context teardown, so the frame must stay readable. */
    MTKVCP_SURF_DECODE_STAGED,
};

/* Decode surface states. */
enum mtkvcp_surf_state {
    MTKVCP_SS_IDLE = 0,   /* buffer queued in driver, no picture */
    MTKVCP_SS_TARGET,     /* bound as render target of an in-flight picture */
    MTKVCP_SS_READY,      /* holds a completed frame, dequeued */
    MTKVCP_SS_ERROR,
};

struct mtkvcp_pending_pic {
    int      surface;     /* render_target index */
    uint64_t seq;
    uint64_t last_submit_ms;  /* CLOCK_MONOTONIC ms of last QBUF */         /* OUTPUT timestamp cookie */
    int      in_use;
};

struct mtkvcp_surface {
    int in_use;
    enum mtkvcp_surf_kind kind;
    enum mtkvcp_surf_state state;
    int width, height;    /* client-visible size */
    int ctx;              /* owning decode/encode context, -1 = none */
    int cap_index;        /* V4L2 CAPTURE buffer index (DECODE kind) */
    int prime_fd;         /* unbound export allocation fd, -1 = none */
    unsigned int fourcc;  /* requested layout, including unbound probes */
    int export_stride, export_bh; /* unbound allocation geometry */
    int export_uvoff;       /* reported NV12 chroma byte offset */
    void *enc_data;       /* raw frame bytes (ENC_INPUT / VPP output) */
    size_t enc_size;
    int enc_stride;
    int enc_rgb;          /* enc_data content: 1 = packed 32-bit RGB,
                           * 0 = NV12, -1 = nothing written yet */
    /* PRIME import (PRIME_IMPORT kind): dup'd dma-buf + layer geometry.
     * Only single-object, single-layer, LINEAR imports are accepted. */
    int imp_fd;                 /* -1 = none */
    unsigned int imp_drm;       /* DRM fourcc of layer 0 */
    int imp_stride;             /* layer-0 plane-0 pitch (bytes) */
    int imp_offset;             /* layer-0 plane-0 byte offset */
    int imp_chroma_off;         /* NV12 second-plane byte offset */
    size_t imp_size;            /* object size (map length) */
    void *imp_map;              /* lazily mmapped object, NULL = unmapped */
    unsigned long long imp_key; /* dma-buf identity: inode of the object */
    int imp_refs;               /* client surface handles referring here */
    int imp_cache;              /* drv import-cache slot + 1, 0 = none */
    /* Cached answer to "is this fd a real dma-buf?". The test is a
     * DMA_BUF_IOCTL_SYNC round trip, which walks the whole scatter list
     * (1.2 ms on a 2460x1080 frame), and the answer cannot change for the
     * lifetime of the import. -1 = not probed yet. */
    int imp_dmabuf;
    /* Zero-copy hand-off: when the post-processing step would only copy an
     * imported dma-buf into its own staging, the buffer can instead be
     * passed straight to the encoder through V4L2 DMABUF memory. These
     * carry that buffer on the output surface. alias_fd is a dup owned by
     * the surface; -1 means the surface must be filled the usual way. */
    int alias_fd;
    int alias_stride;
    int alias_size;
    unsigned long long alias_key; /* imported object identity (F6 affinity) */
    int enc_busy;          /* encoder still owns this input */
};

/* Generic VA buffer object. */
struct mtkvcp_buffer {
    int in_use;
    int ctx;              /* owning context, -1 = none/display-wide */
    unsigned int type;    /* VABufferType */
    unsigned int size;    /* bytes per element */
    unsigned int num_elements;
    void *data;           /* owned bytes (NULL for aliases) */
    void *map_alias;      /* non-owned bytes returned by MapBuffer */
    /* encode coded-buffer sideband */
    int is_coded_seg;     /* data is VACodedBufferSegment, data_ptr is bytes */
    void *coded_bytes;
    size_t coded_size;    /* valid bytes */
    size_t coded_capacity; /* allocated bytes at coded_bytes */
    unsigned int coded_status;
    int coded_busy;        /* an encode job will still write this buffer */
};

/* One submitted-but-not-yet-retired encode frame. OUTPUT and CAPTURE
 * completions are independent events: a job is only reusable once both the
 * input slot and the coded bytes have come back. */
struct mtkvcp_enc_job {
    int in_use;
    uint64_t seq;          /* OUTPUT timestamp cookie */
    int surface;           /* input surface, input ownership */
    int coded_buf;         /* destination coded buffer */
    int out_slot;          /* OUTPUT ring slot */
    int alias_fd;          /* dup of the aliased dma-buf, or -1 */
    uint64_t submit_ms;
    int output_done;       /* OUTPUT buffer returned */
    int coded_done;        /* CAPTURE payload copied and recycled */
    int status;            /* VAStatus once failed */
};

struct mtkvcp_image {
    int in_use;
    unsigned int format_fourcc;
    int width, height;
    void *data;           /* CPU bytes (CreateImage) or surface mmap (Derive) */
    size_t size;
    int derived_surface;  /* >=0 if derived, -1 if owned */
    int buf;              /* backing VABufferID */
};

struct mtkvcp_config {
    int in_use;
    int profile;
    int entrypoint;
    unsigned int rt_format;
};

/* ---- capture-path profiling (MTK_VCP_VA_PROF) ----
 *
 * The frame path has several stages whose cost is not visible from the
 * achieved rate alone: the VideoProc step, the dma-buf capability probe,
 * the encoder submit, and the wait for the coded frame. Each is timed
 * separately and reported once a second, so a slow chain can be attributed
 * to a specific stage instead of guessed at.
 */
enum mtkvcp_prof_stage {
    MTKVCP_PROF_VPP_TOTAL = 0, /* whole VideoProc EndPicture */
    MTKVCP_PROF_VPP_PROBE,     /* dma-buf capability probe ioctls */
    MTKVCP_PROF_VPP_MAP,       /* mmap of the imported buffer */
    MTKVCP_PROF_VPP_DUP,       /* fcntl dup for the zero-copy alias */
    MTKVCP_PROF_ENC_REAP,      /* draining finished OUTPUT buffers */
    MTKVCP_PROF_ENC_COPY,      /* userspace copy into the OUTPUT ring */
    MTKVCP_PROF_ENC_QBUF,      /* submit ioctl */
    MTKVCP_PROF_ENC_WAIT,      /* poll until the coded frame lands */
    MTKVCP_PROF_ENC_RECYCLE,   /* coded CAPTURE buffer re-queue */
    MTKVCP_PROF_ENC_TOTAL,     /* whole encode EndPicture */
    MTKVCP_PROF_ENC_INPUTWAIT, /* alias frame: wait for input to be released */
    MTKVCP_PROF_STAGES
};

struct mtkvcp_context {
    int in_use;
    int config;           /* config index */
    int is_encode;
    int is_vpp;           /* CPU-side video post-processing (no VCP) */
    int vfd;              /* V4L2 fd, -1 = closed */
    int streaming;        /* STREAMON done (== HW owner held) */
    int error;            /* sticky: DRC or HW failure, recreate */
    /* geometry */
    int width, height;
    unsigned int out_fourcc;  /* coded (dec) / NV12 (enc) */
    unsigned int cap_fourcc;  /* NV12/P010 (dec) / coded (enc) */
    int out_count, cap_count;
    int pool_size;            /* fixed CAPTURE pool (codec/resolution) */
    int out_stride;           /* OUTPUT bytesperline (encode raw-in) */
    int enc_visible_width, enc_visible_height; /* OUTPUT crop used by VENC */
    int enc_padded_uv;    /* private V4L2 NV12 source layout selected */
    /* The capture path hands the encoder an imported dma-buf directly, so
     * the OUTPUT queue runs in V4L2 DMABUF memory instead of MMAP. A vb2
     * queue has one memory type, so this is chosen at create time. */
    int out_dmabuf;
    int cap_stride;           /* CAPTURE bytesperline (decode raw-out) */
    int cap_bh;               /* CAPTURE buffer height (stride rows) */
    int rgb_in;               /* packed 32-bit RGB staging (VPP output and
                               * encoder raw input): the firmware converts */
    /* Where the time goes in the capture path: the VPP copy and the
     * encoder copy are the two userspace copies a frame makes before it
     * reaches the driver, so they are measured separately. */
    uint64_t vpp_copy_us_total;
    uint64_t vpp_copy_us_max;
    uint64_t vpp_copies;
    uint64_t enc_copy_us_total;
    uint64_t enc_copy_us_max;
    /* OUTPUT ring */
    void *out_map[MTKVCP_OUT_BUFS_DEC];
    size_t out_len[MTKVCP_OUT_BUFS_DEC];
    int out_free[MTKVCP_OUT_BUFS_DEC];
    int out_next;
    /* CAPTURE (decode): pool mirrors the client's render targets */
    int cap_mmap_count;
    void *cap_map[MTKVCP_MAX_SURFACES];
    size_t cap_len[MTKVCP_MAX_SURFACES];
    int cap_export[MTKVCP_MAX_SURFACES]; /* cached fd + 1, zero if absent */
    /* encode: CAPTURE ring for coded output */
    void *enccap_map[MTKVCP_CAP_BUFS_ENC];
    size_t enccap_len[MTKVCP_CAP_BUFS_ENC];
    int enccap_free[MTKVCP_CAP_BUFS_ENC];
    /* Per-slot dma-heap staging for frames without an alias (DMABUF ring).
     * One allocation per slot, reused every time the slot is refilled. */
    int out_stage_fd[MTKVCP_OUT_BUFS_ENC];
    void *out_stage_map[MTKVCP_OUT_BUFS_ENC];
    size_t out_stage_len[MTKVCP_OUT_BUFS_ENC];
    unsigned long long out_slot_key[MTKVCP_OUT_BUFS_ENC]; /* alias affinity */
    /* pending decode pictures, FIFO */
    struct mtkvcp_pending_pic pend[MTKVCP_MAX_SURFACES];
    int pend_head, pend_tail;
    uint64_t seq;
    uint64_t last_submit_ms;  /* CLOCK_MONOTONIC ms of last QBUF */
    /* pending encode picture assembly */
    uint8_t *enc_au;      /* raw frame being assembled (from surface) */
    size_t enc_au_size;
    int enc_target;       /* input surface index */
    int enc_coded_buf;    /* destination VA buffer index */
    int enc_force_idr;    /* seen IDR slice param this picture */
    int enc_seq_written;  /* sequence params applied */
    /* asynchronous completion queue (F1/F2): EndPicture submits and
     * returns; MapBuffer/SyncBuffer wait for their own job. */
    struct mtkvcp_enc_job enc_jobs[MTKVCP_ENC_MAX_JOBS];
    int enc_outstanding;
    int enc_params_dirty;  /* dynamic rate/fps/GOP not yet applied */
    unsigned int enc_applied_bitrate, enc_applied_gop;
    unsigned int enc_applied_fps_num, enc_applied_fps_den;
    unsigned int enc_operation_rate;   /* client's true rate, sent via OP_RATE */
    unsigned int enc_applied_op_rate;
    /* Frame-rate accounting for the capture path: how many frames the
     * encoder actually completes, and how long each one waited. The
     * stream rate is decided by krdp from network feedback, so the only
     * way to know whether 60 fps is being reached is to count here. */
    uint64_t enc_frames;
    uint64_t enc_first_ms;
    uint64_t enc_last_ms;
    uint64_t enc_wait_us_total;
    uint64_t enc_wait_us_max;
    uint64_t enc_last_report_ms;
    /* Per-stage capture profiling. Enabled by MTK_VCP_VA_PROF; reports once
     * a second so every stage of the frame path is visible in milliseconds
     * rather than inferred from the aggregate rate. Indexed by
     * enum mtkvcp_prof_stage; prof_max keeps the worst sample per stage. */
    uint64_t prof_frames;
    uint64_t prof_last_report_ms;
    uint64_t prof_us[MTKVCP_PROF_STAGES];
    uint64_t prof_max_us[MTKVCP_PROF_STAGES];
    /* encode params stash */
    unsigned int enc_bitrate;
    int enc_bitrate_explicit;    /* client supplied bits_per_second */
    unsigned int enc_quality_factor; /* ICQ factor from the client, 0 = none */
    unsigned int enc_framerate_num, enc_framerate_den;
    unsigned int enc_gop;
    /* current decode picture assembly */
    uint8_t *au;
    size_t au_len, au_cap;
    int au_target;        /* -1 = none */
    int au_has_pic_param;
    int au_error;         /* sticky render error (bad offset, scaling) */
    int custom_scaling;   /* client sent non-flat IQ matrices */
    /* H264 parameter-set synthesis inputs (copied at render time;
     * client buffers may die before EndPicture). */
    VAPictureParameterBufferH264 h264_pic;
    int h264_pic_valid;
    uint32_t h264_ref_l0, h264_ref_l1;
    int h264_slice_seen;
    VAPictureParameterBufferHEVC hevc_pic;
    int hevc_pic_valid;
    VAPictureParameterBufferMPEG2 mp2_pic;
    int mp2_pic_valid;
    VAIQMatrixBufferMPEG2 mp2_iq;
    int mp2_iq_valid;
    int mp2_ps_seen;      /* firmware already holds sequence headers */
    /* Display-order temporal refs without lookahead: N belief (Bs per
     * GOP, init 2 = IBBP), pending Bs since last anchor, last anchor
     * ref. Absolute origin may shift on relock; gaps stay exact, which
     * is what prediction needs. */
    int mp2_N;
    int mp2_pending;
    int mp2_last_anchor;
    int mp2_anchored_once; /* first inter-anchor interval: N unknown */
    int slice_offs[40];   /* AU offsets of slice NAL starts */
    int nslices;
    /* Mid-stream drain recovery. The firmware reset that resumes after a
     * flush discards the DPB, so the AUs from the last IDR are re-submitted
     * to rebuild it before the stream continues. Replayed pictures go out
     * with timestamp 0 and no pend record, so their completions are
     * unattributed and simply requeued. */
    uint8_t *replay_au[MTKVCP_REPLAY_MAX];
    size_t   replay_len[MTKVCP_REPLAY_MAX];
    int      replay_target[MTKVCP_REPLAY_MAX];   /* AU render target surface */
    int      replay_nref[MTKVCP_REPLAY_MAX];     /* H.264 refs captured */
    int      replay_ref[MTKVCP_REPLAY_MAX][16];  /* referenced surface idx */
    int      replay_next;      /* ring write position */
    int      replay_count;     /* valid entries */
    int      replay_idr;       /* ring index of the last IDR, -1 = none */
    /* drain */
    int stop_sent;
    int saw_last;
    int negotiated;       /* CAPTURE rebuilt at parsed geometry */
    int completed_one;    /* ≥1 picture attributed: firmware is warm.
                           * Gates the EOS tail-flush heuristics below:
                           * a cold VCP can stall seconds on first frames,
                           * which must never look like end-of-stream. */
    /* VPP picture assembly (CPU convert/scale, no VCP involved) */
    int vpp_target;       /* output surface index, -1 = none */
    VAProcPipelineParameterBuffer vpp_params;
    int vpp_has_params;
};

struct mtkvcp_drv {
    pthread_mutex_t lock;
    struct mtkvcp_config configs[MTKVCP_MAX_CONTEXTS];
    struct mtkvcp_context contexts[MTKVCP_MAX_CONTEXTS];
    struct mtkvcp_surface surfaces[MTKVCP_MAX_SURFACES];
    struct mtkvcp_buffer buffers[MTKVCP_MAX_BUFFERS];
    struct mtkvcp_image images[MTKVCP_MAX_IMAGES];
    int hw_owner_ctx;     /* context index holding the VCP session, -1 */
    int hw_owner_enc;     /* 1 if owner is an encoder (informational) */
    /* dma-buf probe/mmap cache shared by every surface that wraps the same
     * imported object (F5): PipeWire recycles a handful of buffers but asks
     * for a fresh VA surface each time, and the old per-surface cache died
     * with it. */
    struct {
        int in_use;
        int refs;
        unsigned long long key;   /* object identity (inode) */
        size_t size;
        int is_dmabuf;             /* -1 = unknown */
        void *map;
        size_t map_size;
        uint64_t used_ms;
    } import_cache[32];
    int enc_dmabuf_ring;     /* encoder OUTPUT queue runs in DMABUF mode */
};

const char *mtkvcp_va_status_str(int status);
void mtkvcp_log(const char *fmt, ...);

/* Monotonic microseconds. Shared by the capture-path profiling below. */
uint64_t mtkvcp_now_us(void);
/* 1 when MTK_VCP_VA_PROF is set: emit the once-a-second stage report. */
int mtkvcp_prof_enabled(void);
/* Fold one stage sample into the per-context counters. */
void mtkvcp_prof_stage(struct mtkvcp_context *c, int which, uint64_t us);
/* Count one completed frame and flush the once-a-second stage report. */
void mtkvcp_prof_frame(struct mtkvcp_context *c, const char *tag);

/* Packed-RGB encoder input (see mtkvcp_rgb_input_mode). */
#define MTKVCP_RGB_NONE   0
#define MTKVCP_RGB_ABGR32 1   /* B,G,R,X = DRM_FORMAT_ARGB8888 */
#define MTKVCP_RGB_ARGB32 2   /* A,R,G,B */
int mtkvcp_rgb_input_mode(void);
int mtkvcp_zerocopy_enabled(void);
unsigned int mtkvcp_rgb_fourcc(int mode);
int mtkvcp_rgb_stride(int width);

/* parameter-set synthesis (va_mtkvcp_ps.c) */
int mtkvcp_h264_build_ps(const VAPictureParameterBufferH264 *pic,
                         int va_profile, int w, int h,
                         const int *pps_ids, int nids,
                         uint32_t ref_l0, uint32_t ref_l1,
                         uint8_t *out, size_t cap);
int mtkvcp_h264_nal_pps_id(const uint8_t *nal, size_t len);
int mtkvcp_hevc_build_ps(const VAPictureParameterBufferHEVC *pic,
                         int profile_idc, int w, int h,
                         const int *pps_ids, int nids,
                         uint8_t *out, size_t cap);
int mtkvcp_hevc_nal_pps_id(const uint8_t *nal, size_t len);
int mtkvcp_mpeg2_build_ps(const VAPictureParameterBufferMPEG2 *pic,
                          int va_profile,
                          const uint8_t *iq_intra,
                          const uint8_t *iq_nonintra, int temporal_ref,
                          uint8_t *out, size_t cap);
extern const uint8_t mtkvcp_mpeg2_intra_default[64];

/* cross-TU entry points (caller holds d->lock) */
VAStatus mtkvcp_dec_sync(struct mtkvcp_drv *d, int si, uint64_t timeout_ns);
int mtkvcp_alloc_image(struct mtkvcp_drv *d);
void mtkvcp_surface_release(struct mtkvcp_drv *d, int si);
VAStatus mtkvcp_vpp_import_surface(struct mtkvcp_drv *d, int w, int h,
    const VADRMPRIMESurfaceDescriptor *prime, VASurfaceID *out);
void mtkvcp_vpp_release_surface(struct mtkvcp_drv *d, int si);
VAStatus mtkvcp_vpp_import_surface_v1(struct mtkvcp_drv *d, int w, int h,
    const VASurfaceAttribExternalBuffers *ext, VASurfaceID *out);

/* encode completion pump (va_mtkvcp_enc.c). Caller holds d->lock; the wait
 * helper releases it while polling so other contexts and VPP keep moving. */
void mtkvcp_enc_pump(struct mtkvcp_drv *d, int ci);
VAStatus mtkvcp_enc_wait_job(struct mtkvcp_drv *d, int ci, uint64_t seq,
    uint64_t timeout_ns);
VAStatus mtkvcp_enc_wait_buffer(struct mtkvcp_drv *d, int bi,
    uint64_t timeout_ns);
int mtkvcp_enc_buffer_busy(struct mtkvcp_drv *d, int bi);
int mtkvcp_enc_surface_busy(struct mtkvcp_drv *d, int si);
int mtkvcp_padded_uv_enabled(void);
VAStatus mtkvcp_enc_sync_surface(struct mtkvcp_drv *d, int si,
    uint64_t timeout_ns);

/* v4l2 helpers (va_mtkvcp_v4l2.c) */
int mtkvcp_xioctl(int fd, unsigned long req, void *arg);
int mtkvcp_v4l2_open(const char *node);
int mtkvcp_v4l2_s_fmt(int fd, enum v4l2_buf_type type, uint32_t fourcc,
                      int w, int h, size_t *sizeimage_out);
int mtkvcp_v4l2_reqbufs(int fd, enum v4l2_buf_type type, int count);
int mtkvcp_v4l2_stream(int fd, enum v4l2_buf_type type, int on);
int mtkvcp_v4l2_subscribe(int fd, uint32_t evtype);
int mtkvcp_dma_heap_alloc(size_t size);
int mtkvcp_v4l2_reqbufs_mem(int fd, enum v4l2_buf_type type, int count,
                            uint32_t memory);
int mtkvcp_fd_is_dmabuf(int fd);
int mtkvcp_dmabuf_cpu_read(int fd);

#endif /* VA_MTKVCP_H */
