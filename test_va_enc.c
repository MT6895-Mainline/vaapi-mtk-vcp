/* SPDX-License-Identifier: MIT */
/* VAAPI encode harness: synthetic NV12 in, Annex-B H264/HEVC out.
 * Usage: test_va_enc [--hevc] [--frames N] [--w W] [--h H] [--gop G]
 *                    [--force-idr K] [--bitrate BPS] <out.264|out.265>
 * Exit 0 iff every frame produces coded bytes.
 * Frame K (if given) is submitted as I-slice to test force-IDR. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>

#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_enc_h264.h>
#include <va/va_enc_hevc.h>

#define DIE(...) do { fprintf(stderr, "FAIL: " __VA_ARGS__); \
                      fprintf(stderr, "\n"); return 1; } while (0)
#define CHECKST(fn, st) do { if ((st) != VA_STATUS_SUCCESS) { \
    fprintf(stderr, "FAIL: %s -> %d\n", fn, st); return 1; } } while (0)

static void gen_nv12(uint8_t *dst, int w, int h, int f)
{
    int x, y;
    for (y = 0; y < h; y++)
        for (x = 0; x < w; x++)
            dst[y * w + x] = (uint8_t)((x + y * 2 + f * 7) & 0xff);
    {
        uint8_t *uv = dst + w * h;
        for (y = 0; y < h / 2; y++)
            for (x = 0; x < w; x += 2) {
                uv[y * w + x] = (uint8_t)(128 + ((x + f * 13) & 0x7f));
                uv[y * w + x + 1] = (uint8_t)(128 - ((y + f * 5) & 0x7f));
            }
    }
}

int main(int argc, char **argv)
{
    int hevc = 0, nframes = 20, w = 320, h = 256, gop = 30;
    int force_idr = 10, bitrate = 1000000;
    int fps = 0;   /* 0 = leave the backend default */
    const char *out_path = NULL;
    int i;
    int drmfd, major = 0, minor = 0;
    VADisplay dpy;
    VAStatus st;
    VAProfile profile;
    VAConfigID config;
    VAContextID context;
    VASurfaceID surf;
    VAImageFormat imgfmt;
    FILE *ofp;
    uint8_t *raw;
    int frames_ok = 0;
    size_t total = 0;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--hevc"))
            hevc = 1;
        else if (!strcmp(argv[i], "--frames") && i + 1 < argc)
            nframes = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--w") && i + 1 < argc)
            w = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--h") && i + 1 < argc)
            h = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--gop") && i + 1 < argc)
            gop = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--force-idr") && i + 1 < argc)
            force_idr = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--bitrate") && i + 1 < argc)
            bitrate = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--fps") && i + 1 < argc)
            fps = atoi(argv[++i]);
        else
            out_path = argv[i];
    }
    if (!out_path || (w % 2) || (h % 2) || nframes <= 0) {
        fprintf(stderr, "usage: %s [--hevc] [--frames N] [--w W] [--h H] "
                "[--gop G] [--force-idr K] [--bitrate BPS] <out>\n",
                argv[0]);
        return 2;
    }
    profile = hevc ? VAProfileHEVCMain : VAProfileH264High;
    raw = malloc((size_t)w * h * 3 / 2);
    if (!raw)
        DIE("oom");
    ofp = fopen(out_path, "wb");
    if (!ofp)
        DIE("open output");

    drmfd = open("/dev/dri/renderD128", O_RDWR);
    if (drmfd < 0)
        DIE("open render node");
    dpy = vaGetDisplayDRM(drmfd);
    if (!dpy)
        DIE("vaGetDisplayDRM");
    st = vaInitialize(dpy, &major, &minor);
    CHECKST("vaInitialize", st);
    st = vaCreateConfig(dpy, profile, VAEntrypointEncSlice, NULL, 0,
                        &config);
    CHECKST("vaCreateConfig", st);
    st = vaCreateSurfaces(dpy, VA_RT_FORMAT_YUV420, w, h, &surf, 1, NULL,
                          0);
    CHECKST("vaCreateSurfaces", st);
    st = vaCreateContext(dpy, config, w, h, VA_PROGRESSIVE, &surf, 1,
                         &context);
    CHECKST("vaCreateContext", st);
    imgfmt.fourcc = VA_FOURCC_NV12;
    imgfmt.byte_order = VA_LSB_FIRST;
    imgfmt.bits_per_pixel = 12;

    for (i = 0; i < nframes; i++) {
        VABufferID seq = VA_INVALID_ID, pic = VA_INVALID_ID;
        VABufferID slice = VA_INVALID_ID, coded = VA_INVALID_ID;
        VABufferID misc = VA_INVALID_ID;
        VAImage img;
        void *mp;
        int is_i = (i == force_idr);
        VACodedBufferSegment *seg;

        gen_nv12(raw, w, h, i);
        /* Upload raw frame. */
        st = vaCreateImage(dpy, &imgfmt, w, h, &img);
        CHECKST("vaCreateImage", st);
        st = vaMapBuffer(dpy, img.buf, &mp);
        CHECKST("vaMapBuffer(img)", st);
        memcpy(mp, raw, (size_t)w * h * 3 / 2);
        st = vaUnmapBuffer(dpy, img.buf);
        CHECKST("vaUnmapBuffer(img)", st);
        st = vaBeginPicture(dpy, context, surf);
        CHECKST("vaBeginPicture", st);
        st = vaPutImage(dpy, surf, img.image_id, 0, 0, w, h, 0, 0, w,
                        h);
        CHECKST("vaPutImage", st);
        st = vaDestroyImage(dpy, img.image_id);
        CHECKST("vaDestroyImage", st);

        /* Sequence + rate control + picture + slice params. */
        if (hevc) {
            VAEncSequenceParameterBufferHEVC s;
            VAEncPictureParameterBufferHEVC p;
            VAEncSliceParameterBufferHEVC sp;
            memset(&s, 0, sizeof(s));
            s.intra_period = (uint32_t)gop;
            s.bits_per_second = (uint32_t)bitrate;
            st = vaCreateBuffer(dpy, context,
                                VAEncSequenceParameterBufferType,
                                sizeof(s), 1, &s, &seq);
            CHECKST("seq", st);
            st = vaRenderPicture(dpy, context, &seq, 1);
            CHECKST("render seq", st);
            memset(&p, 0, sizeof(p));
            st = vaCreateBuffer(dpy, context, VAEncCodedBufferType,
                                1 << 20, 1, NULL, &coded);
            CHECKST("coded", st);
            p.coded_buf = coded;
            st = vaCreateBuffer(dpy, context,
                                VAEncPictureParameterBufferType,
                                sizeof(p), 1, &p, &pic);
            CHECKST("pic", st);
            st = vaRenderPicture(dpy, context, &pic, 1);
            CHECKST("render pic", st);
            memset(&sp, 0, sizeof(sp));
            sp.slice_type = is_i ? 2 : 1;
            sp.num_ctu_in_slice = (uint32_t)(w / 64) * (uint32_t)(h / 64);
            st = vaCreateBuffer(dpy, context,
                                VAEncSliceParameterBufferType, sizeof(sp),
                                1, &sp, &slice);
            CHECKST("slice", st);
            st = vaRenderPicture(dpy, context, &slice, 1);
            CHECKST("render slice", st);
        } else {
            VAEncSequenceParameterBufferH264 s;
            VAEncPictureParameterBufferH264 p;
            VAEncSliceParameterBufferH264 sp;
            VAEncMiscParameterBuffer *m;
            VAEncMiscParameterRateControl *rc;
            uint8_t mbuf[sizeof(*m) + sizeof(*rc)];
            memset(&s, 0, sizeof(s));
            s.intra_period = (uint32_t)gop;
            s.intra_idr_period = (uint32_t)gop;
            s.bits_per_second = (uint32_t)bitrate;
            st = vaCreateBuffer(dpy, context,
                                VAEncSequenceParameterBufferType,
                                sizeof(s), 1, &s, &seq);
            CHECKST("seq", st);
            st = vaRenderPicture(dpy, context, &seq, 1);
            CHECKST("render seq", st);
            memset(mbuf, 0, sizeof(mbuf));
            m = (VAEncMiscParameterBuffer *)mbuf;
            m->type = VAEncMiscParameterTypeRateControl;
            rc = (VAEncMiscParameterRateControl *)m->data;
            rc->bits_per_second = (uint32_t)bitrate;
            rc->target_percentage = 100;
            st = vaCreateBuffer(dpy, context,
                                VAEncMiscParameterBufferType,
                                sizeof(mbuf), 1, mbuf, &misc);
            CHECKST("misc", st);
            st = vaRenderPicture(dpy, context, &misc, 1);
            CHECKST("render misc", st);
            if (fps > 0) {
                /* Explicit frame rate: krdp passes one, and the firmware
                 * uses it to pace frame completion, so it decides the
                 * achievable rate. */
                uint8_t fbuf[sizeof(VAEncMiscParameterBuffer) +
                             sizeof(VAEncMiscParameterFrameRate)];
                VAEncMiscParameterBuffer *fm = (VAEncMiscParameterBuffer *)fbuf;
                VAEncMiscParameterFrameRate *fr =
                    (VAEncMiscParameterFrameRate *)fm->data;
                VABufferID fbuf_id = VA_INVALID_ID;
                memset(fbuf, 0, sizeof(fbuf));
                fm->type = VAEncMiscParameterTypeFrameRate;
                fr->framerate = (uint32_t)fps;
                st = vaCreateBuffer(dpy, context,
                                    VAEncMiscParameterBufferType,
                                    sizeof(fbuf), 1, fbuf, &fbuf_id);
                CHECKST("misc fps", st);
                st = vaRenderPicture(dpy, context, &fbuf_id, 1);
                CHECKST("render misc fps", st);
                vaDestroyBuffer(dpy, fbuf_id);
            }
            memset(&p, 0, sizeof(p));
            st = vaCreateBuffer(dpy, context, VAEncCodedBufferType,
                                1 << 20, 1, NULL, &coded);
            CHECKST("coded", st);
            p.coded_buf = coded;
            st = vaCreateBuffer(dpy, context,
                                VAEncPictureParameterBufferType,
                                sizeof(p), 1, &p, &pic);
            CHECKST("pic", st);
            st = vaRenderPicture(dpy, context, &pic, 1);
            CHECKST("render pic", st);
            memset(&sp, 0, sizeof(sp));
            sp.slice_type = is_i ? 2 : 0;
            sp.num_macroblocks = (uint32_t)(w / 16) * (uint32_t)(h / 16);
            if (is_i)
                sp.idr_pic_id = 1;
            st = vaCreateBuffer(dpy, context,
                                VAEncSliceParameterBufferType, sizeof(sp),
                                1, &sp, &slice);
            CHECKST("slice", st);
            st = vaRenderPicture(dpy, context, &slice, 1);
            CHECKST("render slice", st);
        }
        st = vaEndPicture(dpy, context);
        CHECKST("vaEndPicture", st);
        st = vaSyncBuffer(dpy, coded, 0);
        CHECKST("vaSyncBuffer", st);
        st = vaMapBuffer(dpy, coded, (void **)&seg);
        CHECKST("vaMapBuffer(coded)", st);
        if (!seg || !seg->buf || !seg->size) {
            fprintf(stderr, "FAIL: empty coded segment frame %d\n", i);
            return 1;
        }
        {
            unsigned int nbytes = seg->size;
            int is_i_frame = is_i;
            fwrite(seg->buf, 1, nbytes, ofp);
            total += nbytes;
            st = vaUnmapBuffer(dpy, coded);
            CHECKST("vaUnmapBuffer(coded)", st);
            fprintf(stderr, "enc frame %d ok bytes=%u%s\n", i, nbytes,
                    is_i_frame ? " (I)" : "");
        }
        st = vaDestroyBuffer(dpy, coded);
        CHECKST("vaDestroyBuffer(coded)", st);
        if (seq != VA_INVALID_ID)
            vaDestroyBuffer(dpy, seq);
        if (pic != VA_INVALID_ID)
            vaDestroyBuffer(dpy, pic);
        if (slice != VA_INVALID_ID)
            vaDestroyBuffer(dpy, slice);
        if (misc != VA_INVALID_ID)
            vaDestroyBuffer(dpy, misc);
        frames_ok++;
    }
    fclose(ofp);
    st = vaDestroyContext(dpy, context);
    CHECKST("vaDestroyContext", st);
    st = vaDestroySurfaces(dpy, &surf, 1);
    CHECKST("vaDestroySurfaces", st);
    st = vaDestroyConfig(dpy, config);
    CHECKST("vaDestroyConfig", st);
    st = vaTerminate(dpy);
    CHECKST("vaTerminate", st);
    close(drmfd);
    free(raw);
    printf("ENC-OK frames=%d total=%zu\n", frames_ok, total);
    return frames_ok == nframes ? 0 : 1;
}
