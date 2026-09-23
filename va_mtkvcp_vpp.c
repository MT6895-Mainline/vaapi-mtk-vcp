/* SPDX-License-Identifier: MIT */
/* Video post-processing: PRIME import + CPU convert/scale (no VCP).
 *
 * Serves the KPipeWire/krdp hardware path, which is:
 *   hwmap=mode=direct:derive_device=vaapi   (import PipeWire dma-buf)
 *   scale_vaapi=format=nv12:mode=fast       (convert to encoder NV12)
 *   h264_vaapi                              (existing EncSlice path)
 *
 * Only single-object, single-layer, LINEAR imports are accepted
 * (XRGB/ARGB/XBGR/ABGR/NV12). Conversion is BT.601/BT.709 full-range
 * RGB to limited-range NV12; same-size NV12 is a plain copy. VPP
 * contexts never touch the VCP session lock, so VPP + encode run in
 * the same process the way FFmpeg drives them.
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>
#include <libdrm/drm_fourcc.h>

#include <va/va.h>
#include <va/va_backend.h>
#include <va/va_vpp.h>
#include <va/va_drmcommon.h>

#include "va_mtkvcp.h"

static uint64_t mtkvcp_vpp_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

/* ---- import ---- */

/* Identity of the underlying dma-buf behind an fd: the inode number is
 * shared by every fd referring to the same buffer, while the fd number
 * itself is not. */
static unsigned long long mtkvcp_vpp_fd_key(int fd)
{
    struct stat st;
    if (fstat(fd, &st) < 0)
        return 0;
    return (unsigned long long)st.st_ino;
}

static int mtkvcp_vpp_rgb_kind(unsigned int drm)
{
    switch (drm) {
    case DRM_FORMAT_XRGB8888:
    case DRM_FORMAT_ARGB8888:
    case DRM_FORMAT_XBGR8888:
    case DRM_FORMAT_ABGR8888:
        return 1;
    default:
        return 0;
    }
}

/*
 * The legacy PRIME descriptor carries a VA fourcc while the PRIME_2
 * descriptor carries a DRM fourcc for the very same buffer, so translate
 * before validating. VA and DRM order the bytes the same way (VA_FOURCC_
 * BGRA is DRM_FORMAT_ARGB8888 on little-endian), the difference is only
 * which channel the name starts from.
 */
static unsigned int mtkvcp_vpp_va_to_drm(unsigned int va_fourcc)
{
    switch (va_fourcc) {
    case VA_FOURCC_BGRA: return DRM_FORMAT_ARGB8888;
    case VA_FOURCC_BGRX: return DRM_FORMAT_XRGB8888;
    case VA_FOURCC_RGBA: return DRM_FORMAT_ABGR8888;
    case VA_FOURCC_RGBX: return DRM_FORMAT_XBGR8888;
    case VA_FOURCC_ARGB: return DRM_FORMAT_BGRA8888;
    case VA_FOURCC_XRGB: return DRM_FORMAT_BGRX8888;
    case VA_FOURCC_ABGR: return DRM_FORMAT_RGBA8888;
    case VA_FOURCC_XBGR: return DRM_FORMAT_RGBX8888;
    case VA_FOURCC_NV12: return DRM_FORMAT_NV12;
    case VA_FOURCC_NV21: return DRM_FORMAT_NV21;
    default: return va_fourcc;
    }
}

/* Shared tail of both import flavours: validate the layer geometry,
 * dup the dma-buf and register the surface slot. */
/* ---- imported dma-buf object cache (F5) ----
 *
 * PipeWire cycles a handful of physical buffers but asks for a fresh VA
 * surface per frame; the probe/mmap knowledge must survive the surface that
 * happened to discover it. Entries are keyed by object identity + size and
 * live in the driver instance, with unreferenced entries kept in a bounded
 * LRU so the next import reuses the answers for free.
 */
static void mtkvcp_import_cache_free(struct mtkvcp_drv *d, int idx)
{
    if (idx < 0 || idx >= 32 || !d->import_cache[idx].in_use)
        return;
    if (d->import_cache[idx].map)
        munmap(d->import_cache[idx].map, d->import_cache[idx].map_size);
    memset(&d->import_cache[idx], 0, sizeof(d->import_cache[idx]));
    d->import_cache[idx].is_dmabuf = -1;
}

static int mtkvcp_import_cache_get(struct mtkvcp_drv *d,
                                   unsigned long long key, size_t size)
{
    int i, best = -1;

    for (i = 0; i < 32; i++)
        if (d->import_cache[i].in_use && d->import_cache[i].key == key &&
            d->import_cache[i].size == size) {
            d->import_cache[i].refs++;
            d->import_cache[i].used_ms = mtkvcp_now_us() / 1000ull;
            return i;
        }
    for (i = 0; i < 32; i++)
        if (!d->import_cache[i].in_use)
            break;
    if (i == 32) {
        for (i = 0; i < 32; i++) {
            if (d->import_cache[i].refs)
                continue;
            if (best < 0 || d->import_cache[i].used_ms <
                            d->import_cache[best].used_ms)
                best = i;
        }
        if (best < 0)
            return -1;
        mtkvcp_import_cache_free(d, best);
        i = best;
    }
    d->import_cache[i].in_use = 1;
    d->import_cache[i].refs = 1;
    d->import_cache[i].key = key;
    d->import_cache[i].size = size;
    d->import_cache[i].is_dmabuf = -1;
    d->import_cache[i].map = NULL;
    d->import_cache[i].map_size = 0;
    d->import_cache[i].used_ms = mtkvcp_now_us() / 1000ull;
    return i;
}

static void mtkvcp_import_cache_put(struct mtkvcp_drv *d, int idx)
{
    if (idx < 0 || idx >= 32 || !d->import_cache[idx].in_use)
        return;
    if (d->import_cache[idx].refs > 0)
        d->import_cache[idx].refs--;
    /* Unreferenced entries stay cached for the LRU window. */
}

static VAStatus mtkvcp_vpp_import_common(struct mtkvcp_drv *d, int w, int h,
    int fd, uint32_t drm, uint32_t size, int pitch0, int off0,
    int num_planes, int pitch1, int chroma_off, VASurfaceID *out)
{
    int si, dupfd, rgb;
    size_t need;
    unsigned long long key;
    if (w < MTKVCP_MIN_W || h < MTKVCP_MIN_H ||
        w > MTKVCP_MAX_W || h > MTKVCP_MAX_H)
        return VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED;
    if (fd < 0 || size == 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    rgb = mtkvcp_vpp_rgb_kind(drm);
    if (!rgb && drm != DRM_FORMAT_NV12)
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    if (pitch0 <= 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    need = (size_t)off0 + (size_t)pitch0 * (size_t)h;
    if (rgb) {
        if (num_planes != 1)
            return VA_STATUS_ERROR_INVALID_PARAMETER;
    } else {
        if (num_planes != 2 || pitch1 <= 0)
            return VA_STATUS_ERROR_INVALID_PARAMETER;
        need = (size_t)chroma_off + (size_t)pitch1 * (size_t)(h / 2);
    }
    if (need > size)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    /* PipeWire cycles a small pool of dma-bufs (three, typically) while
     * ffmpeg asks for a fresh VA surface on every hwmap. Wrapping the
     * same buffer in a new surface each time burns through the surface
     * pool and starts failing with MAX_NUM_EXCEEDED partway through a
     * session. Both surfaces would alias the same memory anyway, so
     * hand back the existing slot for a buffer already imported with
     * identical geometry, and count the extra reference. */
    key = mtkvcp_vpp_fd_key(fd);
    if (key) {
        int i;
        for (i = 0; i < MTKVCP_MAX_SURFACES; i++) {
            struct mtkvcp_surface *e = &d->surfaces[i];
            if (!e->in_use || e->kind != MTKVCP_SURF_PRIME_IMPORT ||
                e->imp_refs <= 0 || e->imp_key != key)
                continue;
            if (e->imp_drm != drm || e->imp_stride != pitch0 ||
                e->imp_offset != off0 || e->imp_chroma_off != chroma_off ||
                e->width != w || e->height != h)
                continue;
            e->imp_refs++;
            *out = (VASurfaceID)(i + 1);
            return VA_STATUS_SUCCESS;
        }
    }
    dupfd = fcntl(fd, F_DUPFD_CLOEXEC, 0);
    if (dupfd < 0)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    si = -1;
    {
        int i;
        for (i = 0; i < MTKVCP_MAX_SURFACES; i++)
            if (!d->surfaces[i].in_use) {
                d->surfaces[i].in_use = 1;
                si = i;
                break;
            }
    }
    if (si < 0) {
        close(dupfd);
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }
    memset(&d->surfaces[si], 0, sizeof(d->surfaces[si]));
    d->surfaces[si].in_use = 1;
    d->surfaces[si].kind = MTKVCP_SURF_PRIME_IMPORT;
    d->surfaces[si].state = MTKVCP_SS_IDLE;
    d->surfaces[si].width = w;
    d->surfaces[si].height = h;
    d->surfaces[si].ctx = -1;
    d->surfaces[si].cap_index = -1;
    d->surfaces[si].prime_fd = -1;
    d->surfaces[si].fourcc = 0;
    d->surfaces[si].enc_data = NULL;
    d->surfaces[si].imp_fd = dupfd;
    d->surfaces[si].imp_drm = drm;
    d->surfaces[si].imp_stride = pitch0;
    d->surfaces[si].imp_offset = off0;
    d->surfaces[si].imp_chroma_off = chroma_off;
    d->surfaces[si].imp_size = size;
    d->surfaces[si].imp_map = NULL;
    d->surfaces[si].imp_key = key;
    d->surfaces[si].imp_refs = 1;
    d->surfaces[si].alias_fd = -1;
    d->surfaces[si].alias_key = 0;
    d->surfaces[si].imp_dmabuf = -1;
    d->surfaces[si].imp_cache = 0;
    if (key) {
        int e = mtkvcp_import_cache_get(d, key, size);

        if (e >= 0) {
            d->surfaces[si].imp_cache = e + 1;
            if (d->import_cache[e].is_dmabuf >= 0)
                d->surfaces[si].imp_dmabuf =
                    d->import_cache[e].is_dmabuf;
        }
    }
    *out = (VASurfaceID)(si + 1);
    mtkvcp_log("import %dx%d drm=0x%x -> %d (fd=%d ino=%llu)", w, h, drm,
               si + 1, fd, (unsigned long long)d->surfaces[si].imp_key);
    return VA_STATUS_SUCCESS;
}

VAStatus mtkvcp_vpp_import_surface(struct mtkvcp_drv *d, int w, int h,
    const VADRMPRIMESurfaceDescriptor *prime, VASurfaceID *out)
{
    int rgb, chroma_off = 0;
    int fd, pitch0, off0, pitch1 = 0;
    uint32_t drm, size;
    uint64_t modifier;
    if (!prime || !out)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (prime->num_objects != 1 || prime->num_layers != 1)
        return VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE;
    fd = prime->objects[0].fd;
    size = prime->objects[0].size;
    modifier = prime->objects[0].drm_format_modifier;
    drm = prime->layers[0].drm_format;
    pitch0 = (int)prime->layers[0].pitch[0];
    off0 = (int)prime->layers[0].offset[0];
    pitch1 = (int)prime->layers[0].pitch[1];
    /* The descriptor's size field is not reliable: PipeWire hands over a
     * dma-buf whose rows are pitched to an aligned width (2460-pixel
     * frames arrive with pitch 9856 = 2464 * 4) while the size it
     * reports is computed from the unaligned width (2460 * 4 * 1080),
     * i.e. smaller than pitch * height. Trusting that field rejected
     * every krdp frame with UNSUPPORTED_MEMORY_TYPE. Ask the kernel for
     * the real length of the object instead and fall back to the
     * descriptor only when the fd cannot answer. */
    {
        off_t real = lseek(fd, 0, SEEK_END);
        if (real > 0 && (uint64_t)real > size)
            size = (uint32_t)real;
    }
    if (fd < 0 || size == 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (modifier != DRM_FORMAT_MOD_LINEAR) {
        mtkvcp_log("import rejected: modifier=0x%llx is not LINEAR",
                   (unsigned long long)modifier);
        return VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE;
    }
    rgb = mtkvcp_vpp_rgb_kind(drm);
    if (prime->layers[0].object_index[0] != 0) {
        mtkvcp_log("import rejected: object_index[0]=%u",
                   prime->layers[0].object_index[0]);
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }
    if (prime->width && (int)prime->width != w)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (prime->height && (int)prime->height != h)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (!rgb && (prime->layers[0].num_planes != 2 ||
                 prime->layers[0].object_index[1] != 0)) {
            mtkvcp_log("import rejected: NV12 planes=%u objidx1=%u",
                       prime->layers[0].num_planes,
                       prime->layers[0].object_index[1]);
            return VA_STATUS_ERROR_INVALID_PARAMETER;
    }
    chroma_off = (int)prime->layers[0].offset[1];
    return mtkvcp_vpp_import_common(d, w, h, fd, drm, size, pitch0, off0,
                                    (int)prime->layers[0].num_planes,
                                    pitch1, chroma_off, out);
}

/*
 * Legacy VASurfaceAttribExternalBuffers flavour (VA_SURFACE_ATTRIB_MEM_TYPE_
 * DRM_PRIME). ffmpeg falls back to it when the PRIME_2 attempt fails, and
 * KPipeWire hits that fallback for some of its buffers, so both must work.
 * The dma-buf fd arrives as the single entry of ext->buffers and the pitch
 * and offset of each plane are in fixed arrays. There is no modifier field:
 * this descriptor only ever describes linear buffers.
 */
VAStatus mtkvcp_vpp_import_surface_v1(struct mtkvcp_drv *d, int w, int h,
    const VASurfaceAttribExternalBuffers *ext, VASurfaceID *out)
{
    uint32_t drm, size;
    int fd, planes, chroma_off = 0;
    if (!ext || !out)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (ext->num_buffers != 1 || !ext->buffers)
        return VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE;
    fd = (int)ext->buffers[0];
    size = ext->data_size;
    drm = mtkvcp_vpp_va_to_drm(ext->pixel_format);
    planes = (int)ext->num_planes;
    if (planes < 1 || planes > 3)
        return VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE;
    if (planes >= 2)
        chroma_off = (int)ext->offsets[1];
    if (ext->width && (int)ext->width != w)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (ext->height && (int)ext->height != h)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    /* Like the PRIME_2 path, the reported size is derived from the
     * unaligned width and can be smaller than pitch * height. */
    {
        off_t real = lseek(fd, 0, SEEK_END);
        if (real > 0 && (uint64_t)real > size)
            size = (uint32_t)real;
    }
    return mtkvcp_vpp_import_common(d, w, h, fd, drm, size,
                                    (int)ext->pitches[0],
                                    (int)ext->offsets[0], planes,
                                    (int)ext->pitches[1], chroma_off, out);
}

/* Lazily mmap the imported object (caller holds d->lock). */
static const uint8_t *mtkvcp_vpp_map(struct mtkvcp_drv *d, int si)
{
    struct mtkvcp_surface *s = &d->surfaces[si];
    void *m;
    int e = s->imp_cache - 1;

    if (s->imp_map)
        return (const uint8_t *)s->imp_map;
    if (e >= 0 && d->import_cache[e].map) {
        s->imp_map = d->import_cache[e].map;
        return (const uint8_t *)s->imp_map;
    }
    m = mmap(NULL, s->imp_size, PROT_READ, MAP_SHARED, s->imp_fd, 0);
    if (m == MAP_FAILED)
        return NULL;
    s->imp_map = m;
    if (e >= 0 && !d->import_cache[e].map) {
        d->import_cache[e].map = m;
        d->import_cache[e].map_size = s->imp_size;
    }
    return (const uint8_t *)m;
}

/* The dma-buf capability probe is two ioctls that the kernel takes a long
 * time over: DMA_BUF_IOCTL_SYNC on an imported buffer walks the whole
 * scatter list to bounce the caches, which measured 0.36 ms at 720p and
 * 1.2 ms at 2460x1080 - it scales with the frame. It only has to be asked
 * once per import, because the answer is a property of the exporter behind
 * the fd and cannot change while the surface lives. Caching it removes a
 * per-frame cost that was pure waste on the zero-copy path, where the
 * probe was the single largest stage of the VideoProc step.
 */
static int mtkvcp_vpp_probe_dmabuf(struct mtkvcp_drv *d,
                                   struct mtkvcp_context *c, int si)
{
    struct mtkvcp_surface *s = &d->surfaces[si];
    uint64_t t0;
    int e;

    if (s->imp_dmabuf >= 0)
        return s->imp_dmabuf;
    e = s->imp_cache - 1;
    if (e >= 0 && d->import_cache[e].is_dmabuf >= 0) {
        s->imp_dmabuf = d->import_cache[e].is_dmabuf;
        return s->imp_dmabuf;
    }
    t0 = mtkvcp_now_us();
    s->imp_dmabuf = mtkvcp_fd_is_dmabuf(s->imp_fd) ? 1 : 0;
    mtkvcp_prof_stage(c, MTKVCP_PROF_VPP_PROBE, mtkvcp_now_us() - t0);
    if (e >= 0)
        d->import_cache[e].is_dmabuf = s->imp_dmabuf;
    return s->imp_dmabuf;
}

void mtkvcp_vpp_release_surface(struct mtkvcp_drv *d, int si)
{
    struct mtkvcp_surface *s = &d->surfaces[si];

    if (s->imp_map) {
        int e = s->imp_cache - 1;

        /* A cached mapping is owned by the object cache, not the surface. */
        if (!(e >= 0 && d->import_cache[e].map == s->imp_map))
            munmap(s->imp_map, s->imp_size);
        s->imp_map = NULL;
    }
    if (s->imp_cache > 0) {
        mtkvcp_import_cache_put(d, s->imp_cache - 1);
        s->imp_cache = 0;
    }
    if (s->imp_fd >= 0) {
        close(s->imp_fd);
        s->imp_fd = -1;
    }
}

/* ---- converters ---- */

/* Full-range RGB to limited-range NV12. */
struct mtkvcp_csc {
    int c00, c01, c02;  /* Y row, >>8 then +16 */
    int c10, c11, c12;  /* U row, >>8 then +128 */
    int c20, c21, c22;  /* V row, >>8 then +128 */
};

static void mtkvcp_csc_pick(int std, struct mtkvcp_csc *m)
{
    if (std == 1) { /* BT.709 */
        m->c00 = 47; m->c01 = 157; m->c02 = 16;
        m->c10 = -26; m->c11 = -87; m->c12 = 113;
        m->c20 = 113; m->c21 = -102; m->c22 = -10;
    } else {        /* BT.601 (also 240M/2020/unknown: same path) */
        m->c00 = 66; m->c01 = 129; m->c02 = 25;
        m->c10 = -38; m->c11 = -74; m->c12 = 112;
        m->c20 = 112; m->c21 = -94; m->c22 = -18;
    }
}

static int mtkvcp_std_of(int color_standard)
{
    switch (color_standard) {
    case VAProcColorStandardBT709:
        return 1;
    default:
        return 0;
    }
}

static void mtkvcp_read_rgb(unsigned int drm, const uint8_t *p,
                            int *r, int *g, int *b)
{
    switch (drm) {
    case DRM_FORMAT_XBGR8888:
    case DRM_FORMAT_ABGR8888:
        *r = p[0]; *g = p[1]; *b = p[2];
        break;
    default: /* XRGB8888 / ARGB8888: B,G,R,(X/A) */
        *b = p[0]; *g = p[1]; *r = p[2];
        break;
    }
}

static int mtkvcp_clamp8(int v)
{
    return v < 0 ? 0 : (v > 255 ? 255 : v);
}

/* Bilinear maps in 8.8 fixed point for one axis: pos/pos2 are the
 * clamped source taps (in absolute coordinates), w1 the weight of
 * the second tap (256 = fully second). */
static void mtkvcp_axis_map(int dn, int sn, int off,
                            int *pos, int *pos2, int *w1)
{
    int i;
    if (sn < 1)
        sn = 1;
    for (i = 0; i < dn; i++) {
        /* source coordinate of destination pixel center */
        int sc = (2 * i + 1) * sn - dn; /* 2*dn units of (src - off) */
        int p = sc / (2 * dn);
        int frac;
        if (p < 0) {
            p = 0;
            frac = 0;
        } else if (p >= sn - 1) {
            p = sn - 1;
            frac = 0;
        } else {
            frac = (sc - p * 2 * dn) * 256 / (2 * dn);
        }
        pos[i] = off + p;
        pos2[i] = off + (p + 1 < sn ? p + 1 : p);
        w1[i] = frac;
    }
}

struct mtkvcp_rect {
    int x, y, w, h;
};

static void mtkvcp_rect_full(const VARectangle *r, int fw, int fh,
                             struct mtkvcp_rect *o)
{
    if (!r) {
        o->x = 0; o->y = 0; o->w = fw; o->h = fh;
        return;
    }
    o->x = r->x < 0 ? 0 : r->x;
    o->y = r->y < 0 ? 0 : r->y;
    o->w = (int)r->width;
    o->h = (int)r->height;
    if (o->x + o->w > fw)
        o->w = fw - o->x;
    if (o->y + o->h > fh)
        o->h = fh - o->y;
    if (o->w < 0)
        o->w = 0;
    if (o->h < 0)
        o->h = 0;
}

/* RGB import -> NV12 output (convert + optional scale). */
static int mtkvcp_blt_rgb_to_nv12(uint8_t *dst, int dst_stride,
                                   int dx, int dy, int dw, int dh,
                                   const uint8_t *src, int src_stride,
                                   int sx, int sy, int sw, int sh,
                                   unsigned int drm,
                                   const struct mtkvcp_csc *m)
{
    int *px = malloc(sizeof(int) * (size_t)(dw > 0 ? dw : 1));
    int *px2 = malloc(sizeof(int) * (size_t)(dw > 0 ? dw : 1));
    int *wx = malloc(sizeof(int) * (size_t)(dw > 0 ? dw : 1));
    int *py = malloc(sizeof(int) * (size_t)(dh > 0 ? dh : 1));
    int *py2 = malloc(sizeof(int) * (size_t)(dh > 0 ? dh : 1));
    int *wy = malloc(sizeof(int) * (size_t)(dh > 0 ? dh : 1));
    int x, y;
    if (!px || !px2 || !wx || !py || !py2 || !wy) {
        free(px); free(px2); free(wx); free(py); free(py2); free(wy);
        return -1;
    }
    mtkvcp_axis_map(dw, sw, sx, px, px2, wx);
    mtkvcp_axis_map(dh, sh, sy, py, py2, wy);
    for (y = 0; y < dh; y++) {
        uint8_t *yl = dst + (size_t)(dy + y) * (size_t)dst_stride + dx;
        int y0 = py[y], y1 = py2[y], wy1 = wy[y], wy0 = 256 - wy1;
        const uint8_t *r0 = src + (size_t)y0 * (size_t)src_stride;
        const uint8_t *r1 = src + (size_t)y1 * (size_t)src_stride;
        for (x = 0; x < dw; x++) {
            int x0 = px[x], x1 = px2[x], wx1 = wx[x], wx0 = 256 - wx1;
            int r, g, b, r00, g00, b00, r01, g01, b01, r10, g10, b10;
            int r11, g11, b11;
            mtkvcp_read_rgb(drm, r0 + (size_t)x0 * 4, &r00, &g00, &b00);
            mtkvcp_read_rgb(drm, r0 + (size_t)x1 * 4, &r01, &g01, &b01);
            mtkvcp_read_rgb(drm, r1 + (size_t)x0 * 4, &r10, &g10, &b10);
            mtkvcp_read_rgb(drm, r1 + (size_t)x1 * 4, &r11, &g11, &b11);
            r = (r00 * wx0 * wy0 + r01 * wx1 * wy0 +
                 r10 * wx0 * wy1 + r11 * wx1 * wy1 + 32768) >> 16;
            g = (g00 * wx0 * wy0 + g01 * wx1 * wy0 +
                 g10 * wx0 * wy1 + g11 * wx1 * wy1 + 32768) >> 16;
            b = (b00 * wx0 * wy0 + b01 * wx1 * wy0 +
                 b10 * wx0 * wy1 + b11 * wx1 * wy1 + 32768) >> 16;
            yl[x] = (uint8_t)mtkvcp_clamp8(
                16 + ((r * m->c00 + g * m->c01 + b * m->c02 + 128) >> 8));
        }
    }
    for (y = 0; y < dh / 2; y++) {
        uint8_t *uvl = dst + (size_t)(dy + dh) * (size_t)dst_stride +
                       (size_t)(dy / 2 + y) * (size_t)dst_stride + dx;
        /* Chroma subsamples the luma taps. */
        int yy = py[y * 2], yy1 = py2[y * 2];
        int wyy1 = wy[y * 2], wyy0 = 256 - wyy1;
        const uint8_t *r0 = src + (size_t)yy * (size_t)src_stride;
        const uint8_t *r1 = src + (size_t)yy1 * (size_t)src_stride;
        for (x = 0; x < dw / 2; x++) {
            int xx = px[x * 2], xx1 = px2[x * 2];
            int wxx1 = wx[x * 2], wxx0 = 256 - wxx1;
            int r, g, b, r00, g00, b00, r01, g01, b01, r10, g10, b10;
            int r11, g11, b11;
            mtkvcp_read_rgb(drm, r0 + (size_t)xx * 4, &r00, &g00, &b00);
            mtkvcp_read_rgb(drm, r0 + (size_t)xx1 * 4, &r01, &g01, &b01);
            mtkvcp_read_rgb(drm, r1 + (size_t)xx * 4, &r10, &g10, &b10);
            mtkvcp_read_rgb(drm, r1 + (size_t)xx1 * 4, &r11, &g11, &b11);
            r = (r00 * wxx0 * wyy0 + r01 * wxx1 * wyy0 +
                 r10 * wxx0 * wyy1 + r11 * wxx1 * wyy1 + 32768) >> 16;
            g = (g00 * wxx0 * wyy0 + g01 * wxx1 * wyy0 +
                 g10 * wxx0 * wyy1 + g11 * wxx1 * wyy1 + 32768) >> 16;
            b = (b00 * wxx0 * wyy0 + b01 * wxx1 * wyy0 +
                 b10 * wxx0 * wyy1 + b11 * wxx1 * wyy1 + 32768) >> 16;
            uvl[x * 2] = (uint8_t)mtkvcp_clamp8(
                128 + ((r * m->c10 + g * m->c11 + b * m->c12 + 128) >> 8));
            uvl[x * 2 + 1] = (uint8_t)mtkvcp_clamp8(
                128 + ((r * m->c20 + g * m->c21 + b * m->c22 + 128) >> 8));
        }
    }
    free(px); free(px2); free(wx); free(py); free(py2); free(wy);
    return 0;
}

/* NV12 import (or UNBOUND NV12 staging) -> NV12 output. */
static int mtkvcp_blt_nv12_to_nv12(uint8_t *dst, int dst_stride,
                                    int dx, int dy, int dw, int dh,
                                    const uint8_t *src, int src_stride,
                                    int sx, int sy, int sw, int sh)
{
    /* Same-size, full-frame copy needs no resampler (F7): the old code ran
     * the full bilinear machinery even when nothing was scaled. */
    if (!dx && !dy && !sx && !sy && dw == sw && dh == sh) {
        const uint8_t *usrc = src + (size_t)src_stride * (size_t)sh;
        uint8_t *udst = dst + (size_t)dst_stride * (size_t)dh;
        int y;

        for (y = 0; y < dh; y++)
            memcpy(dst + (size_t)y * (size_t)dst_stride,
                   src + (size_t)y * (size_t)src_stride, (size_t)dw);
        for (y = 0; y < dh / 2; y++)
            memcpy(udst + (size_t)y * (size_t)dst_stride,
                   usrc + (size_t)y * (size_t)src_stride, (size_t)dw);
        return 0;
    }
    int *px = malloc(sizeof(int) * (size_t)(dw > 0 ? dw : 1));
    int *px2 = malloc(sizeof(int) * (size_t)(dw > 0 ? dw : 1));
    int *wx = malloc(sizeof(int) * (size_t)(dw > 0 ? dw : 1));
    int *py = malloc(sizeof(int) * (size_t)(dh > 0 ? dh : 1));
    int *py2 = malloc(sizeof(int) * (size_t)(dh > 0 ? dh : 1));
    int *wy = malloc(sizeof(int) * (size_t)(dh > 0 ? dh : 1));
    int uvsw = sw / 2, uvdw = dw / 2, uvsh = sh / 2, uvdh = dh / 2;
    int *upx = malloc(sizeof(int) * (size_t)(uvdw > 0 ? uvdw : 1));
    int *upx2 = malloc(sizeof(int) * (size_t)(uvdw > 0 ? uvdw : 1));
    int *uwx = malloc(sizeof(int) * (size_t)(uvdw > 0 ? uvdw : 1));
    int *upy = malloc(sizeof(int) * (size_t)(uvdh > 0 ? uvdh : 1));
    int *upy2 = malloc(sizeof(int) * (size_t)(uvdh > 0 ? uvdh : 1));
    int *uwy = malloc(sizeof(int) * (size_t)(uvdh > 0 ? uvdh : 1));
    const uint8_t *usrc = src + (size_t)src_stride * (size_t)sh;
    uint8_t *udst = dst + (size_t)dst_stride * (size_t)dh;
    int x, y;
    if (!px || !px2 || !wx || !py || !py2 || !wy ||
        !upx || !upx2 || !uwx || !upy || !upy2 || !uwy) {
        free(px); free(px2); free(wx); free(py); free(py2); free(wy);
        free(upx); free(upx2); free(uwx); free(upy); free(upy2);
        free(uwy);
        return -1;
    }
    mtkvcp_axis_map(dw, sw, sx, px, px2, wx);
    mtkvcp_axis_map(dh, sh, sy, py, py2, wy);
    for (y = 0; y < dh; y++) {
        uint8_t *dl = dst + (size_t)(dy + y) * (size_t)dst_stride + dx;
        int y0 = py[y], y1 = py2[y], w1 = wy[y], w0 = 256 - w1;
        const uint8_t *r0 = src + (size_t)y0 * (size_t)src_stride;
        const uint8_t *r1 = src + (size_t)y1 * (size_t)src_stride;
        for (x = 0; x < dw; x++) {
            int x0 = px[x], x1 = px2[x], v1 = wx[x], v0 = 256 - v1;
            dl[x] = (uint8_t)((r0[x0] * v0 * w0 + r0[x1] * v1 * w0 +
                               r1[x0] * v0 * w1 + r1[x1] * v1 * w1 +
                               32768) >> 16);
        }
    }
    /* Chroma plane: uvdw pixels of 2 bytes (U,V), uvdh rows. Taps
     * address chroma pixels; x0/x1 are byte offsets (U of tap). */
    mtkvcp_axis_map(uvdw, uvsw, sx / 2, upx, upx2, uwx);
    mtkvcp_axis_map(uvdh, uvsh, sy / 2, upy, upy2, uwy);
    for (y = 0; y < uvdh; y++) {
        uint8_t *dl = udst + (size_t)(dy / 2 + y) *
                               (size_t)dst_stride + dx;
        int y0 = upy[y], y1 = upy2[y], w1 = uwy[y], w0 = 256 - w1;
        const uint8_t *r0 = usrc + (size_t)y0 * (size_t)src_stride;
        const uint8_t *r1 = usrc + (size_t)y1 * (size_t)src_stride;
        for (x = 0; x < uvdw; x++) {
            int x0 = upx[x] * 2, x1 = upx2[x] * 2;
            int v1 = uwx[x], v0 = 256 - v1;
            dl[x * 2] = (uint8_t)((r0[x0] * v0 * w0 + r0[x1] * v1 * w0 +
                                   r1[x0] * v0 * w1 + r1[x1] * v1 * w1 +
                                   32768) >> 16);
            dl[x * 2 + 1] = (uint8_t)((r0[x0 + 1] * v0 * w0 +
                                       r0[x1 + 1] * v1 * w0 +
                                       r1[x0 + 1] * v0 * w1 +
                                       r1[x1 + 1] * v1 * w1 +
                                       32768) >> 16);
        }
    }
    free(px); free(px2); free(wx); free(py); free(py2); free(wy);
    free(upx); free(upx2); free(uwx); free(upy); free(upy2); free(uwy);
    return 0;
}

/*
 * Packed-RGB passthrough: carry the imported pixels into the output
 * surface unchanged so the encoder firmware can do the RGB-to-YUV
 * conversion (see mtkvcp_rgb_input_mode). Identical geometry is a plain
 * row copy; a scale would need a real RGB resampler, so a differing
 * region is refused rather than silently mis-scaled. The caller has
 * already checked that the destination is large enough.
 */
static int mtkvcp_blt_rgb_passthrough(uint8_t *dst, int dst_stride,
                                      int dw, int dh,
                                      const uint8_t *src, int src_stride,
                                      int sw, int sh)
{
    int y;
    if (dw != sw || dh != sh)
        return -1;
    for (y = 0; y < dh; y++)
        memcpy(dst + (size_t)y * (size_t)dst_stride,
               src + (size_t)y * (size_t)src_stride, (size_t)sw * 4u);
    return 0;
}

/* CPU fallback staging, allocated only when a frame really needs it (F8). */
static int mtkvcp_vpp_alloc_staging(struct mtkvcp_surface *t,
                                    struct mtkvcp_context *c)
{
    size_t need = c->rgb_in ?
        (size_t)mtkvcp_rgb_stride(c->width) * (size_t)c->height :
        (size_t)c->width * (size_t)c->height * 3u / 2u;

    if (t->enc_data && t->enc_size >= need)
        return 0;
    {
        void *nd = malloc(need ? need : 1);

        if (!nd)
            return -1;
        free(t->enc_data);
        t->enc_data = nd;
        t->enc_size = need;
        memset(t->enc_data, 0, need ? need : 1);
    }
    t->enc_stride = c->rgb_in ? mtkvcp_rgb_stride(c->width) : c->width;
    t->enc_rgb = c->rgb_in ? 1 : 0;
    return 0;
}

/* A CPU-written frame supersedes any earlier alias (F12): keeping the old
 * fd would make the encoder submit last frame's pixels. */
static void mtkvcp_vpp_clear_alias(struct mtkvcp_surface *t)
{
    if (t->alias_fd >= 0)
        close(t->alias_fd);
    t->alias_fd = -1;
    t->alias_key = 0;
}

/* ---- picture lifecycle ---- */

VAStatus mtkvcp_vpp_begin(struct mtkvcp_drv *d, int ci, int si)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    struct mtkvcp_surface *s = &d->surfaces[si];
    if (c->error)
        return VA_STATUS_ERROR_ENCODING_ERROR;
    if (c->vpp_target >= 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    /* VPP outputs are NV12 staging surfaces: fresh UNBOUND slots or
     * encoder-bound slots being recycled frame to frame. Never touch
     * kind/ctx here; the encoder binds them itself later. */
    if (s->kind != MTKVCP_SURF_UNBOUND &&
        s->kind != MTKVCP_SURF_ENC_INPUT)
        return VA_STATUS_ERROR_INVALID_SURFACE;
    if (s->width != c->width || s->height != c->height)
        return VA_STATUS_ERROR_INVALID_SURFACE;
    /* In packed-RGB mode the output surface holds 32-bit samples that the
     * encoder will hand to the firmware unconverted, so the staging is
     * four bytes per pixel with the driver's aligned row pitch. Storage is
     * allocated lazily: alias-only surfaces never need CPU pixels (F8). */
    if (c->rgb_in) {
        s->enc_stride = mtkvcp_rgb_stride(c->width);
        s->enc_rgb = 1;
        c->vpp_target = si;
        c->vpp_has_params = 0;
        return VA_STATUS_SUCCESS;
    }
    s->enc_stride = c->width;
    s->enc_rgb = 0;
    c->vpp_target = si;
    c->vpp_has_params = 0;
    return VA_STATUS_SUCCESS;
}

VAStatus mtkvcp_vpp_render(struct mtkvcp_drv *d, int ci, int bi)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    struct mtkvcp_buffer *b = &d->buffers[bi];
    VAProcPipelineParameterBuffer *p;
    int in_si;
    if (c->vpp_target < 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (b->type != VAProcPipelineParameterBufferType)
        return VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE;
    if ((size_t)b->size * (size_t)b->num_elements <
        sizeof(VAProcPipelineParameterBuffer))
        return VA_STATUS_ERROR_INVALID_BUFFER;
    p = (VAProcPipelineParameterBuffer *)b->data;
    in_si = (int)p->surface - 1;
    if (in_si < 0 || in_si >= MTKVCP_MAX_SURFACES ||
        !d->surfaces[in_si].in_use)
        return VA_STATUS_ERROR_INVALID_SURFACE;
    if (p->rotation_state != VA_ROTATION_NONE ||
        p->mirror_state != VA_MIRROR_NONE)
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    if (p->num_filters)
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    /* Blend flags live in the pipeline caps, not per picture. */
    memcpy(&c->vpp_params, p, sizeof(*p));
    c->vpp_has_params = 1;
    return VA_STATUS_SUCCESS;
}

VAStatus mtkvcp_vpp_end(struct mtkvcp_drv *d, int ci)
{
    struct mtkvcp_context *c = &d->contexts[ci];
    struct mtkvcp_surface *t, *in;
    const uint8_t *base;
    struct mtkvcp_rect sr, dr;
    struct mtkvcp_csc m;
    int in_si, rgb;
    int used_alias = 0;
    uint64_t t_copy = 0;
    uint64_t t_frame = 0, t_stage = 0;
    if (c->vpp_target < 0 || !c->vpp_has_params)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (c->error) {
        c->vpp_target = -1;
        c->vpp_has_params = 0;
        return VA_STATUS_ERROR_ENCODING_ERROR;
    }
    t = &d->surfaces[c->vpp_target];
    in_si = (int)c->vpp_params.surface - 1;
    if (in_si < 0 || in_si >= MTKVCP_MAX_SURFACES ||
        !d->surfaces[in_si].in_use) {
        c->vpp_target = -1;
        c->vpp_has_params = 0;
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    in = &d->surfaces[in_si];
    mtkvcp_csc_pick(mtkvcp_std_of(c->vpp_params.output_color_standard),
                    &m);
    if (in->kind == MTKVCP_SURF_PRIME_IMPORT) {
        t_frame = mtkvcp_now_us();
        rgb = mtkvcp_vpp_rgb_kind(in->imp_drm);
        t_stage = mtkvcp_now_us();
        base = mtkvcp_vpp_map(d, in_si);
        mtkvcp_prof_stage(c, MTKVCP_PROF_VPP_MAP, mtkvcp_now_us() - t_stage);
        if (!base) {
            c->vpp_target = -1;
            c->vpp_has_params = 0;
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }
        base += in->imp_offset;
        t_copy = mtkvcp_vpp_now_us();
        mtkvcp_rect_full(c->vpp_params.surface_region, in->width,
                         in->height, &sr);
        mtkvcp_rect_full(c->vpp_params.output_region, t->width,
                         t->height, &dr);
        if (!sr.w || !sr.h || !dr.w || !dr.h) {
            c->vpp_target = -1;
            c->vpp_has_params = 0;
            return VA_STATUS_ERROR_INVALID_PARAMETER;
        }
        if (rgb) {
            if (c->rgb_in) {
                /*
                 * Zero-copy: the encoder can take the imported dma-buf
                 * itself through V4L2 DMABUF memory, so an unwindowed,
                 * unscaled frame never has to be copied. Hand the buffer
                 * to the output surface and let the encoder submit it.
                 * Anything needing a window or a scale still goes through
                 * the pixel copy below.
                 */
                if (!sr.x && !sr.y && !dr.x && !dr.y &&
                    sr.w == dr.w && sr.h == dr.h &&
                    sr.w == in->width && sr.h == in->height &&
                    in->imp_offset == 0 &&
                    d->enc_dmabuf_ring &&
                    mtkvcp_vpp_probe_dmabuf(d, c, in_si)) {
                    int afd;

                    t_stage = mtkvcp_now_us();
                    afd = fcntl(in->imp_fd, F_DUPFD_CLOEXEC, 0);
                    mtkvcp_prof_stage(c, MTKVCP_PROF_VPP_DUP,
                                      mtkvcp_now_us() - t_stage);
                    if (afd < 0) {
                        c->vpp_target = -1;
                        c->vpp_has_params = 0;
                        return VA_STATUS_ERROR_OPERATION_FAILED;
                    }
                    if (t->alias_fd >= 0)
                        close(t->alias_fd);
                    t->alias_fd = afd;
                    t->alias_stride = in->imp_stride;
                    t->alias_size = (int)in->imp_size;
                    t->alias_key = in->imp_key;
                    t->enc_rgb = 1;
                    used_alias = 1;
                    mtkvcp_log("vpp zero-copy alias fd=%d stride=%d "
                               "size=%d", afd, t->alias_stride,
                               t->alias_size);
                    goto alias_done;
                }
                mtkvcp_log("vpp alias rejected: sr=%d,%d %dx%d dr=%d,%d "
                           "%dx%d in=%dx%d off=%d dmabuf=%d",
                           sr.x, sr.y, sr.w, sr.h, dr.x, dr.y, dr.w, dr.h,
                           in->width, in->height, in->imp_offset,
                           mtkvcp_vpp_probe_dmabuf(d, c, in_si));
                if (mtkvcp_vpp_alloc_staging(t, c) ||
                    mtkvcp_blt_rgb_passthrough(t->enc_data, t->enc_stride,
                                               dr.w, dr.h,
                                               base + (size_t)sr.y *
                                                      (size_t)in->imp_stride +
                                                      (size_t)sr.x * 4u,
                                               in->imp_stride,
                                               sr.w, sr.h) < 0) {
                    c->vpp_target = -1;
                    c->vpp_has_params = 0;
                    mtkvcp_log("vpp rgb passthrough refused: src %d,%d "
                               "%dx%d -> dst %d,%d %dx%d",
                               sr.x, sr.y, sr.w, sr.h,
                               dr.x, dr.y, dr.w, dr.h);
                    return VA_STATUS_ERROR_UNIMPLEMENTED;
                }
            } else {
                if (mtkvcp_vpp_alloc_staging(t, c) ||
                    mtkvcp_blt_rgb_to_nv12(t->enc_data, t->enc_stride,
                                           dr.x, dr.y, dr.w, dr.h,
                                           base, in->imp_stride,
                                           sr.x, sr.y, sr.w, sr.h,
                                           in->imp_drm, &m) < 0) {
                    c->vpp_target = -1;
                    c->vpp_has_params = 0;
                    return VA_STATUS_ERROR_ALLOCATION_FAILED;
                }
            }
        } else {
            if (c->rgb_in) {
                c->vpp_target = -1;
                c->vpp_has_params = 0;
                mtkvcp_log("vpp rgb mode got an NV12 import");
                return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
            }
            if (mtkvcp_vpp_alloc_staging(t, c) ||
                mtkvcp_blt_nv12_to_nv12(t->enc_data, t->enc_stride,
                                        dr.x, dr.y, dr.w, dr.h,
                                        base, in->imp_stride,
                                        sr.x, sr.y, sr.w, sr.h) < 0) {
                c->vpp_target = -1;
                c->vpp_has_params = 0;
                return VA_STATUS_ERROR_ALLOCATION_FAILED;
            }
        }
    } else if (in->kind == MTKVCP_SURF_UNBOUND && in->enc_data) {
        if (c->rgb_in) {
            c->vpp_target = -1;
            c->vpp_has_params = 0;
            mtkvcp_log("vpp rgb mode got a CPU-staged NV12 input");
            return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
        }
        /* CPU-staged NV12 (upload path reuse). */
        mtkvcp_rect_full(c->vpp_params.surface_region, in->width,
                         in->height, &sr);
        mtkvcp_rect_full(c->vpp_params.output_region, t->width,
                         t->height, &dr);
        if (!sr.w || !sr.h || !dr.w || !dr.h) {
            c->vpp_target = -1;
            c->vpp_has_params = 0;
            return VA_STATUS_ERROR_INVALID_PARAMETER;
        }
        if (mtkvcp_vpp_alloc_staging(t, c) ||
            mtkvcp_blt_nv12_to_nv12(t->enc_data, t->enc_stride,
                                    dr.x, dr.y, dr.w, dr.h,
                                    in->enc_data, in->enc_stride,
                                    sr.x, sr.y, sr.w, sr.h) < 0) {
            c->vpp_target = -1;
            c->vpp_has_params = 0;
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
    } else {
        c->vpp_target = -1;
        c->vpp_has_params = 0;
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
alias_done:
    if (!used_alias)
        mtkvcp_vpp_clear_alias(t);
    if (in->kind == MTKVCP_SURF_PRIME_IMPORT) {
        uint64_t d_us = mtkvcp_vpp_now_us() - t_copy;

        mtkvcp_prof_stage(c, MTKVCP_PROF_VPP_TOTAL,
                          mtkvcp_now_us() - t_frame);
        c->vpp_copies++;
        c->vpp_copy_us_total += d_us;
        if (d_us > c->vpp_copy_us_max)
            c->vpp_copy_us_max = d_us;
    }
    if (c->vpp_copies % 30 == 0)
        mtkvcp_log("vpp %dx%d -> %dx%d std=%d copies=%llu avg=%.2fms "
                   "max=%.2fms",
                   sr.w, sr.h, dr.w, dr.h,
                   c->vpp_params.output_color_standard,
                   (unsigned long long)c->vpp_copies,
                   (double)c->vpp_copy_us_total / 1000.0 /
                       (double)c->vpp_copies,
                   (double)c->vpp_copy_us_max / 1000.0);
    else
        mtkvcp_log("vpp %dx%d -> %dx%d std=%d", sr.w, sr.h, dr.w, dr.h,
                   c->vpp_params.output_color_standard);
    mtkvcp_prof_frame(c, "vpp");
    c->vpp_target = -1;
    c->vpp_has_params = 0;
    return VA_STATUS_SUCCESS;
}

/* ---- proc capability queries ---- */

VAStatus mtkvcp_vpp_query_filters(VADriverContextP ctx,
    VAContextID context, VAProcFilterType *filters,
    unsigned int *num_filters)
{
    (void)ctx; (void)context; (void)filters;
    /* Scaling travels in the pipeline filter_flags (FAST/HQ/...),
     * not in filter buffers: no VAProcFilterType is advertised. */
    if (!num_filters)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    *num_filters = 0;
    return VA_STATUS_SUCCESS;
}

VAStatus mtkvcp_vpp_query_filter_caps(VADriverContextP ctx,
    VAContextID context, VAProcFilterType type, void *caps,
    unsigned int *num_caps)
{
    (void)ctx; (void)context; (void)type; (void)caps;
    if (!num_caps)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    *num_caps = 0;
    return VA_STATUS_SUCCESS;
}

static VAProcColorStandardType mtkvcp_vpp_standards[] = {
    VAProcColorStandardBT601,
    VAProcColorStandardBT709,
};

static uint32_t mtkvcp_vpp_in_fmts[] = {
    VA_FOURCC_BGRX,
    VA_FOURCC_NV12,
};

static uint32_t mtkvcp_vpp_out_fmts[] = {
    VA_FOURCC_NV12,
};

/*
 * In packed-RGB mode the post-processing output is not NV12 at all: the
 * RGB bytes travel through untouched and the encoder firmware converts
 * them. Advertising the matching VA fourcc here is what lets ffmpeg's
 * scale_vaapi accept format=<fourcc> and build its output pool without
 * the driver silently ignoring the request (an absent format in this
 * list makes ffmpeg fall back to the input format instead of failing).
 * VA_FOURCC_BGRX is the VA name for bytes B,G,R,X - the same memory
 * order as DRM_FORMAT_ARGB8888 and V4L2_PIX_FMT_ABGR32.
 */
static uint32_t mtkvcp_vpp_out_fmts_rgb[] = {
    VA_FOURCC_BGRX,
    VA_FOURCC_NV12,
};

VAStatus mtkvcp_vpp_query_pipeline_caps(VADriverContextP ctx,
    VAContextID context, VABufferID *filters, unsigned int num_filters,
    VAProcPipelineCaps *caps)
{
    (void)ctx; (void)context; (void)filters; (void)num_filters;
    if (!caps)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    memset(caps, 0, sizeof(*caps));
    caps->pipeline_flags = 0;
    caps->filter_flags = 0;
    caps->num_forward_references = 0;
    caps->num_backward_references = 0;
    caps->input_color_standards = mtkvcp_vpp_standards;
    caps->num_input_color_standards = 2;
    caps->output_color_standards = mtkvcp_vpp_standards;
    caps->num_output_color_standards = 2;
    /* Bitmask form: only identity is supported. */
    caps->rotation_flags = (uint32_t)(1u << VA_ROTATION_NONE);
    caps->blend_flags = 0;
    caps->mirror_flags = (uint32_t)(1u << VA_MIRROR_NONE);
    caps->num_additional_outputs = 0;
    caps->num_input_pixel_formats = 2;
    caps->input_pixel_format = mtkvcp_vpp_in_fmts;
    if (mtkvcp_rgb_input_mode() == MTKVCP_RGB_ABGR32) {
        caps->num_output_pixel_formats = 2;
        caps->output_pixel_format = mtkvcp_vpp_out_fmts_rgb;
    } else {
        caps->num_output_pixel_formats = 1;
        caps->output_pixel_format = mtkvcp_vpp_out_fmts;
    }
    caps->max_input_width = MTKVCP_MAX_W;
    caps->max_input_height = MTKVCP_MAX_H;
    caps->min_input_width = MTKVCP_MIN_W;
    caps->min_input_height = MTKVCP_MIN_H;
    caps->max_output_width = MTKVCP_MAX_W;
    caps->max_output_height = MTKVCP_MAX_H;
    caps->min_output_width = MTKVCP_MIN_W;
    caps->min_output_height = MTKVCP_MIN_H;
    return VA_STATUS_SUCCESS;
}
