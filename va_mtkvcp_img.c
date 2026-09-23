/* SPDX-License-Identifier: MIT */
/* VAImages, DeriveImage, Get/PutImage, PRIME export. */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <libdrm/drm_fourcc.h>

#include <va/va.h>
#include <va/va_backend.h>
#include <va/va_drmcommon.h>

#include "va_mtkvcp.h"

int mtkvcp_alloc_image(struct mtkvcp_drv *d)
{
    int i;
    for (i = 0; i < MTKVCP_MAX_IMAGES; i++)
        if (!d->images[i].in_use) {
            d->images[i].in_use = 1;
            return i;
        }
    return -1;
}

void mtkvcp_surface_release(struct mtkvcp_drv *d, int si)
{
    if (d->surfaces[si].prime_fd >= 0) {
        close(d->surfaces[si].prime_fd);
        d->surfaces[si].prime_fd = -1;
    }
    if (d->surfaces[si].kind == MTKVCP_SURF_PRIME_IMPORT)
        mtkvcp_vpp_release_surface(d, si);
    else if (d->surfaces[si].imp_map)
        munmap(d->surfaces[si].imp_map, d->surfaces[si].imp_size);
    free(d->surfaces[si].enc_data);
    if (d->surfaces[si].alias_fd >= 0)
        close(d->surfaces[si].alias_fd);
    memset(&d->surfaces[si], 0, sizeof(d->surfaces[si]));
    d->surfaces[si].ctx = -1;
    d->surfaces[si].cap_index = -1;
    d->surfaces[si].prime_fd = -1;
    d->surfaces[si].imp_fd = -1;
    d->surfaces[si].alias_fd = -1;
}

static int mtkvcp_is_10bit_fourcc(unsigned int fcc)
{
    return fcc == VA_FOURCC_P010;
}

/* VAImage geometry for a visible w*h frame. */
static void mtkvcp_image_geom(unsigned int fcc, int h, int stride,
                              int *num_planes, int *pitches,
                              int *offsets, size_t *total)
{
    if (mtkvcp_is_10bit_fourcc(fcc)) {
        *num_planes = 2;
        pitches[0] = stride * 2;
        offsets[0] = 0;
        pitches[1] = stride * 2;
        offsets[1] = stride * 2 * h;
        *total = (size_t)stride * 2 * h * 3u / 2u;
    } else {
        *num_planes = 2;
        pitches[0] = stride;
        offsets[0] = 0;
        pitches[1] = stride;
        offsets[1] = stride * h;
        *total = (size_t)stride * h * 3u / 2u;
    }
}

VAStatus mtkvcp_QueryImageFormats(VADriverContextP ctx,
    VAImageFormat *formats, int *num)
{
    (void)ctx;
    if (!formats || !num)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    formats[0].fourcc = VA_FOURCC_NV12;
    formats[0].byte_order = VA_LSB_FIRST;
    formats[0].bits_per_pixel = 12;
    formats[1].fourcc = VA_FOURCC_P010;
    formats[1].byte_order = VA_LSB_FIRST;
    formats[1].bits_per_pixel = 24;
    *num = 2;
    return VA_STATUS_SUCCESS;
}

/* Allocate a VABuffer to back an image; returns buffer index or -1. */
static int mtkvcp_image_buffer(struct mtkvcp_drv *d, size_t total, void *data,
                               void *alias)
{
    int bi, i;
    for (i = 0; i < MTKVCP_MAX_BUFFERS; i++)
        if (!d->buffers[i].in_use)
            break;
    if (i == MTKVCP_MAX_BUFFERS)
        return -1;
    bi = i;
    d->buffers[bi].in_use = 1;
    d->buffers[bi].ctx = -1;
    d->buffers[bi].type = VAImageBufferType;
    d->buffers[bi].size = (unsigned int)total;
    d->buffers[bi].num_elements = 1;
    d->buffers[bi].data = data;
    d->buffers[bi].map_alias = alias;
    d->buffers[bi].is_coded_seg = 0;
    d->buffers[bi].coded_bytes = NULL;
    return bi;
}

VAStatus mtkvcp_CreateImage(VADriverContextP ctx, VAImageFormat *f,
    int w, int h, VAImage *image)
{
    struct mtkvcp_drv *d = (struct mtkvcp_drv *)ctx->pDriverData;
    int ii, bi, num_planes, pitches[3], offsets[3];
    size_t total;
    void *data;
    if (!f || !image || w <= 0 || h <= 0)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (f->fourcc != VA_FOURCC_NV12 && f->fourcc != VA_FOURCC_P010)
        return VA_STATUS_ERROR_INVALID_IMAGE_FORMAT;
    mtkvcp_image_geom(f->fourcc, h, w, &num_planes, pitches, offsets,
                      &total);
    data = malloc(total ? total : 1);
    if (!data)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    pthread_mutex_lock(&d->lock);
    ii = mtkvcp_alloc_image(d);
    if (ii < 0) {
        pthread_mutex_unlock(&d->lock);
        free(data);
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }
    /* The image owns the bytes; the backing VABuffer only aliases
     * them for MapBuffer, so teardown frees exactly once. */
    bi = mtkvcp_image_buffer(d, total, NULL, data);
    if (bi < 0) {
        d->images[ii].in_use = 0;
        pthread_mutex_unlock(&d->lock);
        free(data);
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }
    memset(&d->images[ii], 0, sizeof(d->images[ii]));
    d->images[ii].in_use = 1;
    d->images[ii].format_fourcc = f->fourcc;
    d->images[ii].width = w;
    d->images[ii].height = h;
    d->images[ii].data = data;
    d->images[ii].size = total;
    d->images[ii].derived_surface = -1;
    d->images[ii].buf = bi + 1;
    memset(image, 0, sizeof(*image));
    image->image_id = (VAImageID)(ii + 1);
    image->format = *f;
    image->buf = (VABufferID)(bi + 1);
    image->width = w;
    image->height = h;
    image->data_size = (uint32_t)total;
    image->num_planes = num_planes;
    image->pitches[0] = pitches[0];
    image->pitches[1] = pitches[1];
    image->offsets[0] = offsets[0];
    image->offsets[1] = offsets[1];
    mtkvcp_log("CreateImage id=%d buf=%d data=%p total=%zu", ii + 1,
               bi + 1, data, total);
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

VAStatus mtkvcp_DeriveImage(VADriverContextP ctx, VASurfaceID s,
    VAImage *image)
{
    struct mtkvcp_drv *d = (struct mtkvcp_drv *)ctx->pDriverData;
    int si = (int)s - 1, ii, bi, ci, num_planes;
    int pitches[3], offsets[3];
    size_t total;
    VAStatus st;
    VAImageFormat f;
    void *base;
    if (!image)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (si < 0 || si >= MTKVCP_MAX_SURFACES)
        return VA_STATUS_ERROR_INVALID_SURFACE;
    pthread_mutex_lock(&d->lock);
    if (!d->surfaces[si].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    mtkvcp_log("DeriveImage surf=%d kind=%d ctx=%d", si + 1,
               d->surfaces[si].kind, d->surfaces[si].ctx);
    if (d->surfaces[si].kind == MTKVCP_SURF_ENC_INPUT ||
        (d->surfaces[si].kind == MTKVCP_SURF_UNBOUND &&
         d->surfaces[si].ctx < 0)) {
        /* CPU-staged upload surface: back it on demand and map it.
         * The kind is deliberately left UNBOUND so a later BeginPicture
         * still binds it either direction; a later encode BeginPicture
         * reuses the same storage. Reading a pre-decode derived image
         * after the surface decoded is a client aliasing bug. */
        size_t need;
        need = (size_t)d->surfaces[si].width *
               (size_t)d->surfaces[si].height * 3u / 2u;
        if (!d->surfaces[si].enc_data) {
            d->surfaces[si].enc_data = malloc(need ? need : 1);
            if (!d->surfaces[si].enc_data) {
                pthread_mutex_unlock(&d->lock);
                return VA_STATUS_ERROR_ALLOCATION_FAILED;
            }
            memset(d->surfaces[si].enc_data, 0, need ? need : 1);
            d->surfaces[si].enc_size = need;
            d->surfaces[si].enc_stride = d->surfaces[si].width;
        }
        f.fourcc = VA_FOURCC_NV12;
        f.byte_order = VA_LSB_FIRST;
        f.bits_per_pixel = 12;
        base = d->surfaces[si].enc_data;
        num_planes = 2;
        pitches[0] = d->surfaces[si].enc_stride;
        pitches[1] = d->surfaces[si].enc_stride;
        offsets[0] = 0;
        offsets[1] = d->surfaces[si].enc_stride * d->surfaces[si].height;
        total = (size_t)offsets[1] * 3u / 2u;
        ii = mtkvcp_alloc_image(d);
        if (ii < 0) {
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
        }
        bi = mtkvcp_image_buffer(d, total, NULL, base);
        if (bi < 0) {
            d->images[ii].in_use = 0;
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
        }
        memset(&d->images[ii], 0, sizeof(d->images[ii]));
        d->images[ii].in_use = 1;
        d->images[ii].format_fourcc = f.fourcc;
        d->images[ii].width = d->surfaces[si].width;
        d->images[ii].height = d->surfaces[si].height;
        d->images[ii].data = base;
        d->images[ii].size = total;
        d->images[ii].derived_surface = si;
        d->images[ii].buf = bi + 1;
        memset(image, 0, sizeof(*image));
        image->image_id = (VAImageID)(ii + 1);
        image->format = f;
        image->buf = (VABufferID)(bi + 1);
        image->width = d->images[ii].width;
        image->height = d->images[ii].height;
        image->data_size = (uint32_t)total;
        image->num_planes = num_planes;
        image->pitches[0] = pitches[0];
        image->pitches[1] = pitches[1];
        image->offsets[0] = offsets[0];
        image->offsets[1] = offsets[1];
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_SUCCESS;
    }
    ci = d->surfaces[si].ctx;
    if (ci < 0 || ci >= MTKVCP_MAX_CONTEXTS ||
        !d->contexts[ci].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    /* Derive implies completion: wait for the frame first. */
    st = mtkvcp_dec_sync(d, si, 0);
    if (st != VA_STATUS_SUCCESS) {
        pthread_mutex_unlock(&d->lock);
        return st;
    }
    if (d->surfaces[si].cap_index < 0 ||
        d->surfaces[si].cap_index >= d->contexts[ci].cap_mmap_count) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    f.fourcc = d->contexts[ci].cap_fourcc == V4L2_PIX_FMT_P010 ?
               VA_FOURCC_P010 : VA_FOURCC_NV12;
    f.byte_order = VA_LSB_FIRST;
    f.bits_per_pixel = f.fourcc == VA_FOURCC_P010 ? 24 : 12;
    base = d->contexts[ci].cap_map[d->surfaces[si].cap_index];
    /* bytesperline is already bytes (2 per P010 sample). UV starts
     * at the buffer height: firmware rows may exceed visible. */
    num_planes = 2;
    pitches[0] = d->contexts[ci].cap_stride;
    pitches[1] = d->contexts[ci].cap_stride;
    offsets[0] = 0;
    offsets[1] = d->contexts[ci].cap_stride * d->contexts[ci].cap_bh;
    total = (size_t)offsets[1] * 3u / 2u;
    ii = mtkvcp_alloc_image(d);
    if (ii < 0) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }
    bi = mtkvcp_image_buffer(d, total, NULL, base);
    if (bi < 0) {
        d->images[ii].in_use = 0;
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }
    memset(&d->images[ii], 0, sizeof(d->images[ii]));
    d->images[ii].in_use = 1;
    d->images[ii].format_fourcc = f.fourcc;
    d->images[ii].width = d->surfaces[si].width;
    d->images[ii].height = d->surfaces[si].height;
    d->images[ii].data = base;
    d->images[ii].size = total;
    d->images[ii].derived_surface = si;
    d->images[ii].buf = bi + 1;
    memset(image, 0, sizeof(*image));
    image->image_id = (VAImageID)(ii + 1);
    image->format = f;
    image->buf = (VABufferID)(bi + 1);
    image->width = d->images[ii].width;
    image->height = d->images[ii].height;
    image->data_size = (uint32_t)total;
    image->num_planes = num_planes;
    image->pitches[0] = pitches[0];
    image->pitches[1] = pitches[1];
    image->offsets[0] = offsets[0];
    image->offsets[1] = offsets[1];
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

VAStatus mtkvcp_DestroyImage(VADriverContextP ctx, VAImageID id)
{
    struct mtkvcp_drv *d = (struct mtkvcp_drv *)ctx->pDriverData;
    int ii = (int)id - 1, bi;
    if (ii < 0 || ii >= MTKVCP_MAX_IMAGES)
        return VA_STATUS_ERROR_INVALID_IMAGE;
    pthread_mutex_lock(&d->lock);
    if (!d->images[ii].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_IMAGE;
    }
    mtkvcp_log("DestroyImage id=%d derived=%d buf=%d data=%p", ii + 1,
               d->images[ii].derived_surface, d->images[ii].buf,
               d->images[ii].data);
    bi = d->images[ii].buf - 1;
    if (bi >= 0 && bi < MTKVCP_MAX_BUFFERS && d->buffers[bi].in_use) {
        free(d->buffers[bi].data);
        memset(&d->buffers[bi], 0, sizeof(d->buffers[bi]));
    }
    if (d->images[ii].derived_surface < 0)
        free(d->images[ii].data);
    memset(&d->images[ii], 0, sizeof(d->images[ii]));
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

/* Copy visible rect between equal-format frames with own strides. */
static void mtkvcp_copy_rect(uint8_t *dst, int dst_stride,
                             const uint8_t *src, int src_stride,
                             int x, int y, int w, int h, int bpp)
{
    int r;
    const uint8_t *sp = src + (size_t)y * src_stride + (size_t)x * bpp;
    uint8_t *dp = dst + (size_t)y * dst_stride + (size_t)x * bpp;
    for (r = 0; r < h; r++, sp += src_stride, dp += dst_stride)
        memcpy(dp, sp, (size_t)w * bpp);
}

VAStatus mtkvcp_GetImage(VADriverContextP ctx, VASurfaceID s, int x,
    int y, unsigned int w, unsigned int h, VAImageID id)
{
    struct mtkvcp_drv *d = (struct mtkvcp_drv *)ctx->pDriverData;
    int si = (int)s - 1, ii = (int)id - 1, ci, bpp;
    VAStatus st;
    uint8_t *src, *dst;
    if (si < 0 || si >= MTKVCP_MAX_SURFACES)
        return VA_STATUS_ERROR_INVALID_SURFACE;
    if (ii < 0 || ii >= MTKVCP_MAX_IMAGES)
        return VA_STATUS_ERROR_INVALID_IMAGE;
    pthread_mutex_lock(&d->lock);
    if (!d->surfaces[si].in_use ||
        !d->images[ii].in_use || d->images[ii].derived_surface >= 0) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }
    if (d->surfaces[si].kind == MTKVCP_SURF_UNBOUND ||
        d->surfaces[si].kind == MTKVCP_SURF_ENC_INPUT ||
        d->surfaces[si].kind == MTKVCP_SURF_DECODE_STAGED) {
        /* CPU-staged surface (upload/VPP output): copy NV12 out. */
        uint8_t *csrc, *cdst;
        int csstride, cdstride;
        if (!d->surfaces[si].enc_data ||
            d->images[ii].format_fourcc != VA_FOURCC_NV12) {
            pthread_mutex_unlock(&d->lock);
            return d->surfaces[si].enc_data ?
                   VA_STATUS_ERROR_INVALID_IMAGE_FORMAT :
                   VA_STATUS_ERROR_INVALID_PARAMETER;
        }
        if (x < 0 || y < 0 ||
            x + (int)w > d->surfaces[si].width ||
            y + (int)h > d->surfaces[si].height ||
            (int)w > d->images[ii].width ||
            (int)h > d->images[ii].height) {
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_ERROR_INVALID_PARAMETER;
        }
        csrc = d->surfaces[si].enc_data;
        cdst = d->images[ii].data;
        csstride = d->surfaces[si].enc_stride;
        cdstride = d->images[ii].width;
        mtkvcp_copy_rect(cdst, cdstride, csrc, csstride, x, y, (int)w,
                         (int)h, 1);
        mtkvcp_copy_rect(cdst + cdstride * d->images[ii].height,
                         cdstride,
                         csrc + csstride * d->surfaces[si].height,
                         csstride, x, y / 2, (int)w, (int)h / 2, 1);
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_SUCCESS;
    }
    if (d->surfaces[si].kind != MTKVCP_SURF_DECODE) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }
    ci = d->surfaces[si].ctx;
    if (ci < 0 || ci >= MTKVCP_MAX_CONTEXTS ||
        !d->contexts[ci].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    st = mtkvcp_dec_sync(d, si, 0);
    if (st != VA_STATUS_SUCCESS) {
        pthread_mutex_unlock(&d->lock);
        return st;
    }
    if (x < 0 || y < 0 ||
        x + (int)w > d->surfaces[si].width ||
        y + (int)h > d->surfaces[si].height ||
        (int)w > d->images[ii].width || (int)h > d->images[ii].height) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }
    {
        unsigned int expect = d->contexts[ci].cap_fourcc ==
                              V4L2_PIX_FMT_P010 ? VA_FOURCC_P010 :
                              VA_FOURCC_NV12;
        if (d->images[ii].format_fourcc != expect) {
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_ERROR_INVALID_IMAGE_FORMAT;
        }
    }
    bpp = d->images[ii].format_fourcc == VA_FOURCC_P010 ? 2 : 1;
    (void)bpp; /* destination-side unit only; V4L2 strides are bytes */
    if (d->surfaces[si].cap_index < 0 ||
        d->surfaces[si].cap_index >= d->contexts[ci].cap_mmap_count) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    src = d->contexts[ci].cap_map[d->surfaces[si].cap_index];
    dst = d->images[ii].data;
    /* Luma. */
    mtkvcp_copy_rect(dst, d->images[ii].width * bpp, src,
                     d->contexts[ci].cap_stride, x, y, (int)w,
                     (int)h, bpp);
    /* Chroma (half resolution, interleaved). */
    {
        int sstride = d->contexts[ci].cap_stride;
        int dstride = d->images[ii].width * bpp;
        int uvoff_s = sstride * d->contexts[ci].cap_bh;
        int uvoff_d = dstride * d->images[ii].height;
        mtkvcp_copy_rect(dst + uvoff_d, dstride, src + uvoff_s, sstride,
                         x, y / 2, (int)w, (int)h / 2, bpp);
    }
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

VAStatus mtkvcp_PutImage(VADriverContextP ctx, VASurfaceID s,
    VAImageID id, int sx, int sy, unsigned int sw, unsigned int sh,
    int dx, int dy, unsigned int dw, unsigned int dh)
{
    struct mtkvcp_drv *d = (struct mtkvcp_drv *)ctx->pDriverData;
    int si = (int)s - 1, ii = (int)id - 1, bpp = 1;
    uint8_t *src, *dst;
    if (si < 0 || si >= MTKVCP_MAX_SURFACES)
        return VA_STATUS_ERROR_INVALID_SURFACE;
    if (ii < 0 || ii >= MTKVCP_MAX_IMAGES)
        return VA_STATUS_ERROR_INVALID_IMAGE;
    pthread_mutex_lock(&d->lock);
    if (!d->surfaces[si].in_use ||
        (d->surfaces[si].kind != MTKVCP_SURF_ENC_INPUT &&
         d->surfaces[si].kind != MTKVCP_SURF_UNBOUND) ||
        !d->images[ii].in_use || d->images[ii].derived_surface >= 0 ||
        d->images[ii].format_fourcc != VA_FOURCC_NV12) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }
    if (d->surfaces[si].kind == MTKVCP_SURF_UNBOUND &&
        !d->surfaces[si].enc_data) {
        /* Back CPU staging on demand (same as DeriveImage); the kind
         * stays UNBOUND so a later BeginPicture still binds either
         * direction. */
        size_t need = (size_t)d->surfaces[si].width *
                      (size_t)d->surfaces[si].height * 3u / 2u;
        d->surfaces[si].enc_data = malloc(need ? need : 1);
        if (!d->surfaces[si].enc_data) {
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        memset(d->surfaces[si].enc_data, 0, need ? need : 1);
        d->surfaces[si].enc_size = need;
        d->surfaces[si].enc_stride = d->surfaces[si].width;
        d->surfaces[si].enc_rgb = 0;
    }
    if (!d->surfaces[si].enc_data) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }
    if (sx != 0 || sy != 0 || dx != 0 || dy != 0 ||
        sw != dw || sh != dh ||
        (int)sw > d->surfaces[si].width ||
        (int)sh > d->surfaces[si].height ||
        (int)sw > d->images[ii].width ||
        (int)sh > d->images[ii].height) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }
    src = d->images[ii].data;
    dst = d->surfaces[si].enc_data;
    mtkvcp_copy_rect(dst, d->surfaces[si].enc_stride, src,
                     d->images[ii].width * bpp, 0, 0, (int)sw, (int)sh,
                     bpp);
    {
        int sstride = d->images[ii].width * bpp;
        int dstride = d->surfaces[si].enc_stride * bpp;
        mtkvcp_copy_rect(dst + dstride * d->surfaces[si].height,
                         dstride, src + sstride * d->images[ii].height,
                         sstride, 0, 0, (int)sw, (int)sh / 2, bpp);
    }
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

/* Clients probe EGL formats before creating a codec context. Allocate one
 * zeroed CAPTURE buffer without STREAMON; this neither boots nor claims VCP.
 * The exported fd owns the allocation after this temporary queue is closed.
 */
static int mtkvcp_export_unbound(struct mtkvcp_surface *s)
{
    struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE };
    struct v4l2_exportbuffer exp = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
        /* Clients render into the exported object (FFmpeg's vaapi
         * encoder fills its input surfaces through EGL), so the fd
         * has to be writable. */
        .flags = O_RDWR | O_CLOEXEC,
    };
    unsigned int ah = ((unsigned)s->height + 15u) & ~15u;
    unsigned int aw;
    /* Panfrost (the only GPU that can import these buffers for writing,
     * via EGL dmabuf images) rejects plane pitches that are not a
     * multiple of 64 bytes, while the codec nodes pitch 8-bit luma at
     * ALIGN16(width). Request a 64-byte-aligned width so the exported
     * object can be imported at all; the extra columns are padding the
     * encoder crops away. */
    unsigned int px_align = s->fourcc == V4L2_PIX_FMT_P010 ? 32u : 64u;
    aw = ((unsigned)s->width + px_align - 1u) & ~(px_align - 1u);
    int fd = mtkvcp_v4l2_open(MTKVCP_DEC_NODE), ret = -1;

    if (fd < 0)
        return -1;
    /* The decoder node rounds a request down to its own alignment, so
     * asking for the visible size can come back smaller (2460x1080 ->
     * 2448x1072) and the buffer would not cover the surface. Aligned
     * requests pass through unchanged, so ask for the aligned-up size;
     * the padding rows simply become part of the exported object. */
    f.fmt.pix_mp.width = aw;
    f.fmt.pix_mp.height = ah;
    f.fmt.pix_mp.pixelformat = s->fourcc;
    f.fmt.pix_mp.field = V4L2_FIELD_NONE;
    f.fmt.pix_mp.num_planes = 1;
    if (mtkvcp_xioctl(fd, VIDIOC_S_FMT, &f) < 0 ||
        f.fmt.pix_mp.pixelformat != s->fourcc ||
        f.fmt.pix_mp.width < aw ||
        f.fmt.pix_mp.height < ah ||
        mtkvcp_v4l2_reqbufs(fd, f.type, 1) < 1 ||
        mtkvcp_xioctl(fd, VIDIOC_EXPBUF, &exp) < 0)
        goto out;
    mtkvcp_log("export_unbound %dx%d fourcc=%c%c%c%c -> %ux%u bpl=%u",
               s->width, s->height, s->fourcc & 0xff, (s->fourcc >> 8) & 0xff,
               (s->fourcc >> 16) & 0xff, (s->fourcc >> 24) & 0xff,
               f.fmt.pix_mp.width, f.fmt.pix_mp.height,
               f.fmt.pix_mp.plane_fmt[0].bytesperline);
    s->prime_fd = exp.fd;
    s->export_stride = (int)f.fmt.pix_mp.plane_fmt[0].bytesperline;
    s->export_bh = (int)f.fmt.pix_mp.height;
    {
        off_t osz = lseek(exp.fd, 0, SEEK_END);

        if (osz > 0)
            s->imp_size = (size_t)osz;
    }
    ret = 0;
out:
    close(fd);
    return ret;
}

VAStatus mtkvcp_ExportSurfaceHandle(VADriverContextP ctx,
    VASurfaceID s, uint32_t mem_type, uint32_t flags, void *desc)
{
    struct mtkvcp_drv *d = (struct mtkvcp_drv *)ctx->pDriverData;
    VADRMPRIMESurfaceDescriptor *prime =
        (VADRMPRIMESurfaceDescriptor *)desc;
    int si = (int)s - 1, ci, fd, uvoff, ten_bit, stride, bh, base_fd, idx;
    int uvbh;
    unsigned int fourcc;
    off_t object_size;
    VAStatus st;
    if (!desc)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (mem_type != VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2)
        return VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE;
    if ((flags & ~(VA_EXPORT_SURFACE_READ_WRITE |
                   VA_EXPORT_SURFACE_SEPARATE_LAYERS |
                   VA_EXPORT_SURFACE_COMPOSED_LAYERS)) ||
        ((flags & VA_EXPORT_SURFACE_SEPARATE_LAYERS) &&
         (flags & VA_EXPORT_SURFACE_COMPOSED_LAYERS)))
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    /* The access hint is advisory: an exported object is a plain dma-buf
     * either way, and the driver hands out the same read-write mapping for
     * every variant. VA_EXPORT_SURFACE_READ_WRITE is 0x3, so it already
     * carries the WRITE_ONLY bit; rejecting WRITE_ONLY therefore rejected
     * READ_WRITE as well. FFmpeg's vaapi encoder exports its input surfaces
     * with WRITE_ONLY | SEPARATE_LAYERS (it renders into them through EGL),
     * which is how Sunshine's VA-API encoder and any other libavcodec-based
     * client hit this path. */
    if (si < 0 || si >= MTKVCP_MAX_SURFACES)
        return VA_STATUS_ERROR_INVALID_SURFACE;
    pthread_mutex_lock(&d->lock);
    if (!d->surfaces[si].in_use ||
        (d->surfaces[si].kind != MTKVCP_SURF_DECODE &&
         d->surfaces[si].kind != MTKVCP_SURF_UNBOUND)) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    if (d->surfaces[si].kind == MTKVCP_SURF_UNBOUND) {
        if (d->surfaces[si].prime_fd < 0 &&
            mtkvcp_export_unbound(&d->surfaces[si]) < 0) {
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        stride = d->surfaces[si].export_stride;
        bh = d->surfaces[si].export_bh;
        fourcc = d->surfaces[si].fourcc;
        base_fd = d->surfaces[si].prime_fd;
        /* For the opt-in padded NV12 path the EGL writer puts UV at the
         * firmware's coded-luma offset. The encoder selects the matching
         * V4L2 input layout before its first frame; other clients keep
         * the established visible-height layout. */
        uvbh = mtkvcp_padded_uv_enabled() && !
               (bh % 32) && fourcc == V4L2_PIX_FMT_NV12 ?
               bh : d->surfaces[si].height;
        goto exported;
    }
    st = mtkvcp_dec_sync(d, si, 0);
    if (st != VA_STATUS_SUCCESS) {
        pthread_mutex_unlock(&d->lock);
        return st;
    }
    ci = d->surfaces[si].ctx;
    idx = d->surfaces[si].cap_index;
    if (idx < 0 || idx >= d->contexts[ci].cap_count) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    if (!d->contexts[ci].cap_export[idx]) {
        struct v4l2_exportbuffer exp;
        memset(&exp, 0, sizeof(exp));
        exp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        exp.index = (uint32_t)idx;
        exp.plane = 0;
        exp.flags = O_RDONLY | O_CLOEXEC;
        if (ioctl(d->contexts[ci].vfd, VIDIOC_EXPBUF, &exp) < 0) {
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }
        d->contexts[ci].cap_export[idx] = exp.fd + 1;
    }
    base_fd = d->contexts[ci].cap_export[idx] - 1;
    stride = d->contexts[ci].cap_stride;
    bh = d->contexts[ci].cap_bh;
    fourcc = d->contexts[ci].cap_fourcc;
    uvbh = bh;
exported:
    object_size = lseek(base_fd, 0, SEEK_END);
    if (object_size <= 0 || (uint64_t)object_size > UINT32_MAX ||
        stride <= 0 || bh < d->surfaces[si].height ||
        (uint64_t)stride * (bh + (uint64_t)bh / 2) > (uint64_t)object_size) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }
    fd = fcntl(base_fd, F_DUPFD_CLOEXEC, 0);
    if (fd < 0) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }
    ten_bit = fourcc == V4L2_PIX_FMT_P010;
    uvoff = stride * uvbh;
    d->surfaces[si].export_uvoff = uvoff;
    memset(prime, 0, sizeof(*prime));
    prime->fourcc = ten_bit ?
                    VA_FOURCC_P010 : VA_FOURCC_NV12;
    prime->width = (uint32_t)d->surfaces[si].width;
    prime->height = (uint32_t)d->surfaces[si].height;
    prime->num_objects = 1;
    prime->objects[0].fd = fd;
    prime->objects[0].size = (uint32_t)object_size;
    prime->objects[0].drm_format_modifier = DRM_FORMAT_MOD_LINEAR;
    if (flags & VA_EXPORT_SURFACE_SEPARATE_LAYERS) {
        prime->num_layers = 2;
        prime->layers[0].drm_format = ten_bit ? DRM_FORMAT_R16 : DRM_FORMAT_R8;
        prime->layers[1].drm_format = ten_bit ? DRM_FORMAT_GR1616 : DRM_FORMAT_GR88;
        prime->layers[0].num_planes = 1;
        prime->layers[1].num_planes = 1;
        prime->layers[1].offset[0] = (uint32_t)uvoff;
        prime->layers[0].pitch[0] = (uint32_t)stride;
        prime->layers[1].pitch[0] = (uint32_t)stride;
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_SUCCESS;
    }
    prime->num_layers = 1;
    prime->layers[0].drm_format =
        ten_bit ?
        DRM_FORMAT_P010 : DRM_FORMAT_NV12;
    prime->layers[0].num_planes = 2;
    prime->layers[0].object_index[0] = 0;
    prime->layers[0].object_index[1] = 0;
    prime->layers[0].offset[0] = 0;
    prime->layers[0].offset[1] = (uint32_t)uvoff;
    prime->layers[0].pitch[0] = (uint32_t)stride;
    prime->layers[0].pitch[1] = (uint32_t)stride;
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}
