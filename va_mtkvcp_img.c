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
    if (d->surfaces[si].bounce_map)
        munmap(d->surfaces[si].bounce_map, d->surfaces[si].bounce_size);
    if (d->surfaces[si].bounce_fd >= 0)
        close(d->surfaces[si].bounce_fd);
    memset(&d->surfaces[si], 0, sizeof(d->surfaces[si]));
    d->surfaces[si].ctx = -1;
    d->surfaces[si].cap_index = -1;
    d->surfaces[si].prime_fd = -1;
    d->surfaces[si].bounce_fd = -1;
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
    d->buffers[bi].handle_fd = -1;
    d->buffers[bi].handle_size = 0;
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

/* Byte stride of the dma-buf an UNBOUND surface exports. The external
 * API (VLC's EGL converter) takes the pitch from the derived image, and
 * Panfrost rejects plane pitches that are not a multiple of 64 bytes
 * (P010: 32 pixels). Stage and export with that same aligned stride. */
static int mtkvcp_export_stride_for(unsigned int fourcc, int width)
{
    unsigned int px = fourcc == V4L2_PIX_FMT_P010 ? 32u : 64u;
    int stride = (int)(((unsigned int)width + px - 1u) & ~(px - 1u));

    return fourcc == V4L2_PIX_FMT_P010 ? stride * 2 : stride;
}

/* Linearise the firmware's MM21/MT2T tiles. This is the walk the kernel used
 * to do per frame; the tiled path does not pay it, so a client that reads
 * pixels (vaGetImage / vaDeriveImage) has to - that is the whole trade.
 * MM21 is 16x32 luma tiles and 16x16 chroma tiles in raster order.
 */
void mtkvcp_detile_mm21(uint8_t *dst, int dstride, const uint8_t *src,
                        int sstride, int height, int tile_h)
{
    int x, y;

    for (y = 0; y < height; y++)
        for (x = 0; x < sstride; x += 16) {
            int off = (y / tile_h * (sstride / 16) + x / 16) * tile_h * 16;

            memcpy(dst + (size_t)y * dstride + x,
                   src + off + (y % tile_h) * 16, 16);
        }
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
    void *owned = NULL; /* linear expansion a derived image took ownership of */
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
        int stride, ten;
        ten = d->surfaces[si].fourcc == V4L2_PIX_FMT_P010;
        stride = mtkvcp_export_stride_for(d->surfaces[si].fourcc,
                                          d->surfaces[si].width);
        if (!d->surfaces[si].enc_data) {
            need = (size_t)stride *
                   (size_t)d->surfaces[si].height * 3u / 2u;
            d->surfaces[si].enc_data = malloc(need ? need : 1);
            if (!d->surfaces[si].enc_data) {
                pthread_mutex_unlock(&d->lock);
                return VA_STATUS_ERROR_ALLOCATION_FAILED;
            }
            memset(d->surfaces[si].enc_data, 0, need ? need : 1);
            d->surfaces[si].enc_size = need;
            d->surfaces[si].enc_stride = stride;
            d->surfaces[si].enc_rgb = 0;
        }
        /* A surface already backed by the encoder keeps its own layout;
         * report what the staging actually has. */
        stride = d->surfaces[si].enc_stride;
        f.fourcc = ten ? VA_FOURCC_P010 : VA_FOURCC_NV12;
        f.byte_order = VA_LSB_FIRST;
        f.bits_per_pixel = ten ? 24 : 12;
        base = d->surfaces[si].enc_data;
        num_planes = 2;
        pitches[0] = stride;
        pitches[1] = stride;
        offsets[0] = 0;
        offsets[1] = stride * d->surfaces[si].height;
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
    /* A derived image is a linear view, so a tiled frame is expanded now and
     * the image owns the result (DestroyImage frees the buffer's data). */
    if (d->contexts[ci].cap_tiled) {
        int st = d->contexts[ci].cap_stride, bh = d->contexts[ci].cap_bh;
        uint8_t *lin = malloc((size_t)st * bh * 3 / 2);

        if (!lin) {
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        mtkvcp_detile_mm21(lin, st, base, st, bh, 32);
        mtkvcp_detile_mm21(lin + (size_t)st * bh, st,
                           d->contexts[ci].cap_map_uv[
                                   d->surfaces[si].cap_index],
                           st, bh / 2, 16);
        base = lin;
        owned = lin;
    }
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
        free(owned);
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }
    bi = mtkvcp_image_buffer(d, total, owned, base);
    if (bi < 0) {
        free(owned);
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
        if (d->buffers[bi].handle_fd >= 0)
            close(d->buffers[bi].handle_fd);
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
    if (d->contexts[ci].cap_tiled) {
        int stride = d->contexts[ci].cap_stride;
        int bh = d->contexts[ci].cap_bh;
        int dstride = d->images[ii].width * bpp;
        const uint8_t *uv = d->contexts[ci].cap_map_uv[
                                d->surfaces[si].cap_index];

        if (x == 0 && y == 0 && (int)w == d->images[ii].width &&
            (int)h == d->images[ii].height) {
            /* Whole frame: expand straight into the client's image. Doing
             * it via a scratch would read and write every byte twice, which
             * is exactly the cost this path exists to avoid.
             */
            mtkvcp_detile_mm21(dst, dstride, src, stride, bh, 32);
            mtkvcp_detile_mm21((uint8_t *)dst +
                                       (size_t)dstride * d->images[ii].height,
                               dstride, uv, stride, bh / 2, 16);
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_SUCCESS;
        }
        /* Sub-rectangle: expand into the scratch the copy below reads. */
        {
            size_t need = (size_t)stride * bh * 3 / 2;

            if (d->contexts[ci].tile_scratch_sz < need) {
                free(d->contexts[ci].tile_scratch);
                d->contexts[ci].tile_scratch = malloc(need);
                d->contexts[ci].tile_scratch_sz =
                    d->contexts[ci].tile_scratch ? need : 0;
            }
            if (!d->contexts[ci].tile_scratch) {
                pthread_mutex_unlock(&d->lock);
                return VA_STATUS_ERROR_ALLOCATION_FAILED;
            }
            mtkvcp_detile_mm21(d->contexts[ci].tile_scratch, stride, src,
                               stride, bh, 32);
            mtkvcp_detile_mm21((uint8_t *)d->contexts[ci].tile_scratch +
                                       (size_t)stride * bh,
                               stride, uv, stride, bh / 2, 16);
            src = d->contexts[ci].tile_scratch;
        }
    }
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
    int fd, ret = -1;

    /* The exported object is taken from the decoder's CAPTURE queue, so it
     * must come from the very node the decode context uses: resolving each
     * side independently once renumbered /dev/videoN apart would export a
     * buffer the firmware never writes. */
    aw = ((unsigned)s->width + px_align - 1u) & ~(px_align - 1u);
    fd = mtkvcp_v4l2_open(mtkvcp_dec_node());
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

/* ---- dma-buf handle export (vaAcquire/vaReleaseBufferHandle) ---- */

/* Borrowed dma-buf of a surface, exported on first use. Caller holds
 * d->lock. Returns the fd, or -1 when the surface has no dma-buf. */
static int mtkvcp_surface_dmabuf(struct mtkvcp_drv *d, int si)
{
    struct mtkvcp_surface *s = &d->surfaces[si];

    if (s->kind == MTKVCP_SURF_UNBOUND || s->kind == MTKVCP_SURF_ENC_INPUT) {
        if (s->prime_fd < 0 && mtkvcp_export_unbound(s) < 0)
            return -1;
        return s->prime_fd;
    }
    if (s->kind == MTKVCP_SURF_DECODE) {
        struct mtkvcp_context *c;
        int ci = s->ctx, idx = s->cap_index;

        if (ci < 0 || ci >= MTKVCP_MAX_CONTEXTS || !d->contexts[ci].in_use)
            return -1;
        c = &d->contexts[ci];
        if (idx < 0 || idx >= c->cap_count || idx >= MTKVCP_MAX_SURFACES)
            return -1;
        /* Hand the frame out only once decoding into it has completed. */
        if (mtkvcp_dec_sync(d, si, 0) != VA_STATUS_SUCCESS)
            return -1;
        if (!c->cap_export[idx]) {
            struct v4l2_exportbuffer exp;

            memset(&exp, 0, sizeof(exp));
            exp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            exp.index = (uint32_t)idx;
            exp.plane = 0;
            exp.flags = O_RDONLY | O_CLOEXEC;
            if (ioctl(c->vfd, VIDIOC_EXPBUF, &exp) < 0)
                return -1;
            c->cap_export[idx] = exp.fd + 1;
        }
        return c->cap_export[idx] - 1;
    }
    return -1;
}

/* VLC's OpenGL converter derives an image from a pool surface and then
 * calls vaAcquireBufferHandle() on the image's backing VABuffer to check
 * that the surface can be imported into EGL as a dma-buf before it uses
 * it as a render target. The handle is a dup of the surface's export so
 * the client can hold it while VA-API keeps ownership of the surface. */
VAStatus mtkvcp_AcquireBufferHandle(VADriverContextP ctx, VABufferID id,
    VABufferInfo *info)
{
    struct mtkvcp_drv *d = (struct mtkvcp_drv *)ctx->pDriverData;
    int bi = (int)id - 1, ii, si = -1, base, fd;
    off_t size;

    if (!info)
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    if (info->mem_type != VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME)
        return VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE;
    if (bi < 0 || bi >= MTKVCP_MAX_BUFFERS)
        return VA_STATUS_ERROR_INVALID_BUFFER;
    pthread_mutex_lock(&d->lock);
    if (!d->buffers[bi].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
    if (d->buffers[bi].type != VAImageBufferType) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE;
    }
    if (d->buffers[bi].handle_fd >= 0) {
        info->handle = (intptr_t)d->buffers[bi].handle_fd;
        info->mem_size = d->buffers[bi].handle_size;
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_SUCCESS;
    }
    /* The image that owns this buffer names the surface it was derived
     * from; images from vaCreateImage have no surface storage. */
    for (ii = 0; ii < MTKVCP_MAX_IMAGES; ii++) {
        if (d->images[ii].in_use && d->images[ii].buf == (int)id &&
            d->images[ii].derived_surface >= 0) {
            si = d->images[ii].derived_surface;
            break;
        }
    }
    if (si < 0 || si >= MTKVCP_MAX_SURFACES) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE;
    }
    base = mtkvcp_surface_dmabuf(d, si);
    if (base < 0) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }
    size = lseek(base, 0, SEEK_END);
    fd = fcntl(base, F_DUPFD_CLOEXEC, 0);
    if (fd < 0) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    d->buffers[bi].handle_fd = fd;
    d->buffers[bi].handle_size =
        size > 0 && (uint64_t)size <= UINT32_MAX ? (unsigned int)size : 0;
    info->handle = (intptr_t)fd;
    info->mem_size = d->buffers[bi].handle_size;
    mtkvcp_log("AcquireBufferHandle buf=%d si=%d kind=%d fd=%d size=%u",
               (int)id, si + 1, d->surfaces[si].kind, fd,
               d->buffers[bi].handle_size);
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

/* Paired with vaAcquireBufferHandle. Clients release the handle when the
 * EGL images built from it are gone; releasing a buffer that was never
 * acquired (the client's error path) is a no-op. */
VAStatus mtkvcp_ReleaseBufferHandle(VADriverContextP ctx, VABufferID id)
{
    struct mtkvcp_drv *d = (struct mtkvcp_drv *)ctx->pDriverData;
    int bi = (int)id - 1;

    if (bi < 0 || bi >= MTKVCP_MAX_BUFFERS)
        return VA_STATUS_ERROR_INVALID_BUFFER;
    pthread_mutex_lock(&d->lock);
    if (!d->buffers[bi].in_use) {
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
    if (d->buffers[bi].handle_fd >= 0) {
        close(d->buffers[bi].handle_fd);
        d->buffers[bi].handle_fd = -1;
        d->buffers[bi].handle_size = 0;
    }
    pthread_mutex_unlock(&d->lock);
    return VA_STATUS_SUCCESS;
}

/* Copy a decoded CAPTURE frame into a 64-byte aligned dma-buf, cached
 * on the surface. Panfrost refuses EGL imports whose plane pitch is not a
 * multiple of 64 bytes, while some H.264 streams are only 16-byte pitched
 * (2460 -> 2464); the client gets an aligned copy instead of a green or
 * torn picture. Caller holds d->lock. */
static int mtkvcp_bounce_export(struct mtkvcp_drv *d, int si,
                                const uint8_t *src, int stride, int bh)
{
    struct mtkvcp_surface *s = &d->surfaces[si];
    int want = (stride + 63) & ~63;
    int rows = bh + bh / 2, y;
    uint8_t *dst;
    size_t need;

    if (s->bounce_fd >= 0 && s->bounce_stride != want) {
        munmap(s->bounce_map, s->bounce_size);
        s->bounce_map = NULL;
        close(s->bounce_fd);
        s->bounce_fd = -1;
    }
    if (s->bounce_fd < 0) {
        need = (size_t)want * (size_t)bh * 3u / 2u;
        s->bounce_fd = mtkvcp_dma_heap_alloc_system(need);
        if (s->bounce_fd < 0)
            return -1;
        s->bounce_map = mmap(NULL, need, PROT_READ | PROT_WRITE,
                             MAP_SHARED, s->bounce_fd, 0);
        if (s->bounce_map == MAP_FAILED) {
            s->bounce_map = NULL;
            close(s->bounce_fd);
            s->bounce_fd = -1;
            return -1;
        }
        s->bounce_stride = want;
        s->bounce_size = need;
        mtkvcp_log("bounce export: si=%d pitch %d -> %d (%zu bytes)",
                   si, stride, want, need);
    }
    if (!src || !s->bounce_map)
        return -1;
    dst = s->bounce_map;
    for (y = 0; y < rows; y++) {
        memcpy(dst + (size_t)y * want, src + (size_t)y * stride, stride);
        memset(dst + (size_t)y * want + stride, 0,
               (size_t)(want - stride));
    }
    mtkvcp_dmabuf_cpu_write(s->bounce_fd);
    return 0;
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
    off_t object_size, known_size = 0;
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
    if (si < 0 || si >= MTKVCP_MAX_SURFACES) {
        mtkvcp_log("ExportSurfaceHandle rejected: si=%d out of range", si);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    pthread_mutex_lock(&d->lock);
    if (!d->surfaces[si].in_use ||
        (d->surfaces[si].kind != MTKVCP_SURF_DECODE &&
         d->surfaces[si].kind != MTKVCP_SURF_UNBOUND &&
         d->surfaces[si].kind != MTKVCP_SURF_ENC_INPUT)) {
        mtkvcp_log("ExportSurfaceHandle rejected: si=%d in_use=%d kind=%d ctx=%d "
                   "prime_fd=%d flags=%#x", si,
                   d->surfaces[si].in_use, d->surfaces[si].kind,
                   d->surfaces[si].ctx, d->surfaces[si].prime_fd, flags);
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    /* UNBOUND is the storage a client exports to render into; ENC_INPUT is
     * that very same storage once BeginPicture has bound it. FFmpeg's texture
     * encoder (OBS's ffmpeg_vaapi_tex) re-exports a surface after it has been
     * used, so both kinds must hand back the same dma-buf. */
    if (d->surfaces[si].kind == MTKVCP_SURF_UNBOUND ||
        d->surfaces[si].kind == MTKVCP_SURF_ENC_INPUT) {
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
    {
        uint64_t t0 = mtkvcp_now_us();

        st = mtkvcp_dec_sync(d, si, 0);
        if (mtkvcp_now_us() - t0 > 50000)
            mtkvcp_log("export: si=%d state=%d waited %llu ms", si,
                       d->surfaces[si].state,
                       (unsigned long long)(mtkvcp_now_us() - t0) / 1000);
    }
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
    if (d->contexts[ci].cap_tiled) {
        /* Two separate V4L2 planes, each handed out as its own object, with
         * the tile layout carried by the modifier so a consumer that cannot
         * read tiles knows to reject them instead of misreading rows. */
        struct v4l2_exportbuffer exp;
        unsigned int st = d->contexts[ci].cap_stride;
        unsigned int hh = d->contexts[ci].cap_bh;
        int ten = d->contexts[ci].cap_fourcc == V4L2_PIX_FMT_MT2T;
        int uv_fd;
        off_t sz0, sz1;

        if (!d->contexts[ci].cap_export[idx]) {
            memset(&exp, 0, sizeof(exp));
            exp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            exp.index = (uint32_t)idx;
            exp.plane = 0;
            exp.flags = O_RDONLY | O_CLOEXEC;
            if (ioctl(d->contexts[ci].vfd, VIDIOC_EXPBUF, &exp) < 0) {
                mtkvcp_log("export^: EXPBUF plane 0 idx=%d errno=%d",
                           idx, errno);
                pthread_mutex_unlock(&d->lock);
                return VA_STATUS_ERROR_OPERATION_FAILED;
            }
            d->contexts[ci].cap_export[idx] = exp.fd + 1;
        }
        if (!d->contexts[ci].cap_export_uv[idx]) {
            memset(&exp, 0, sizeof(exp));
            exp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            exp.index = (uint32_t)idx;
            exp.plane = 1;
            exp.flags = O_RDONLY | O_CLOEXEC;
            if (ioctl(d->contexts[ci].vfd, VIDIOC_EXPBUF, &exp) < 0) {
                mtkvcp_log("export^: EXPBUF plane 1 idx=%d errno=%d",
                           idx, errno);
                pthread_mutex_unlock(&d->lock);
                return VA_STATUS_ERROR_OPERATION_FAILED;
            }
            d->contexts[ci].cap_export_uv[idx] = exp.fd + 1;
        }
        sz0 = (off_t)st * hh;
        sz1 = sz0 / 2;
        fd = fcntl(d->contexts[ci].cap_export[idx] - 1, F_DUPFD_CLOEXEC, 0);
        uv_fd = fcntl(d->contexts[ci].cap_export_uv[idx] - 1,
                      F_DUPFD_CLOEXEC, 0);
        if (fd < 0 || uv_fd < 0) {
            if (fd >= 0)
                close(fd);
            if (uv_fd >= 0)
                close(uv_fd);
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }
        memset(prime, 0, sizeof(*prime));
        prime->fourcc = ten ? VA_FOURCC_P010 : VA_FOURCC_NV12;
        prime->width = (uint32_t)d->surfaces[si].width;
        prime->height = (uint32_t)d->surfaces[si].height;
        prime->num_objects = 2;
        prime->objects[0].fd = fd;
        prime->objects[0].size = (uint32_t)sz0;
        prime->objects[0].drm_format_modifier = MTKVCP_DRM_MOD_MM21;
        prime->objects[1].fd = uv_fd;
        prime->objects[1].size = (uint32_t)sz1;
        prime->objects[1].drm_format_modifier = MTKVCP_DRM_MOD_MM21;
        if (!(flags & VA_EXPORT_SURFACE_COMPOSED_LAYERS)) {
            prime->num_layers = 2;
            prime->layers[0].drm_format = ten ? DRM_FORMAT_R16
                                              : DRM_FORMAT_R8;
            prime->layers[1].drm_format = ten ? DRM_FORMAT_GR1616
                                              : DRM_FORMAT_GR88;
            prime->layers[0].num_planes = 1;
            prime->layers[1].num_planes = 1;
            prime->layers[0].object_index[0] = 0;
            prime->layers[1].object_index[0] = 1;
            prime->layers[0].pitch[0] = st;
            prime->layers[1].pitch[0] = st;
        } else {
            prime->num_layers = 1;
            prime->layers[0].drm_format = ten ? DRM_FORMAT_P010
                                              : DRM_FORMAT_NV12;
            prime->layers[0].num_planes = 2;
            prime->layers[0].object_index[0] = 0;
            prime->layers[0].object_index[1] = 1;
            prime->layers[0].pitch[0] = st;
            prime->layers[0].pitch[1] = st;
        }
        mtkvcp_log("export^: tiled si=%d idx=%d %ux%u stride=%u bh=%u mod=%#llx",
                   si, idx, prime->width, prime->height, st, hh,
                   (unsigned long long)MTKVCP_DRM_MOD_MM21);
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_SUCCESS;
    }
    if (!d->contexts[ci].cap_export[idx]) {
        struct v4l2_exportbuffer exp;
        memset(&exp, 0, sizeof(exp));
        exp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        exp.index = (uint32_t)idx;
        exp.plane = 0;
        exp.flags = O_RDONLY | O_CLOEXEC;
        if (ioctl(d->contexts[ci].vfd, VIDIOC_EXPBUF, &exp) < 0) {
            mtkvcp_log("export fail: EXPBUF idx=%d errno=%d", idx, errno);
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
    /* Panfrost refuses EGL imports whose plane pitch is not a multiple
     * of 64 bytes; some H.264 streams are only 16-byte pitched (2460 ->
     * 2464). Hand the client an aligned copy instead of a broken image. */
    if (stride & 63) {
        if (mtkvcp_bounce_export(d, si,
                                 d->contexts[ci].cap_map[idx],
                                 stride, bh) < 0) {
            mtkvcp_log("export fail: bounce si=%d stride=%d bh=%d map=%p",
                       si, stride, bh,
                       d->contexts[ci].cap_map[idx]);
            pthread_mutex_unlock(&d->lock);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }
        base_fd = d->surfaces[si].bounce_fd;
        stride = d->surfaces[si].bounce_stride;
        known_size = (off_t)d->surfaces[si].bounce_size;
    }
exported:
    /* dma-heap objects do not answer lseek(), so the bounce carries its
     * own size. */
    object_size = known_size > 0 ? known_size : lseek(base_fd, 0, SEEK_END);
    if (object_size <= 0 || (uint64_t)object_size > UINT32_MAX ||
        stride <= 0 || bh < d->surfaces[si].height ||
        (uint64_t)stride * (bh + (uint64_t)bh / 2) > (uint64_t)object_size) {
        mtkvcp_log("export fail: size=%lld stride=%d bh=%d h=%d bounce=%d",
                   (long long)object_size, stride, bh,
                   d->surfaces[si].height, d->surfaces[si].bounce_fd >= 0);
        pthread_mutex_unlock(&d->lock);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }
    fd = fcntl(base_fd, F_DUPFD_CLOEXEC, 0);
    if (fd < 0) {
        mtkvcp_log("export fail: dup base_fd=%d errno=%d", base_fd, errno);
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
    /* Clients that want composed layers say so explicitly; both VLC and
     * Mesa leave the layer flags clear and expect one single-plane layer
     * per plane, so separate is the default. */
    if (!(flags & VA_EXPORT_SURFACE_COMPOSED_LAYERS)) {
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
