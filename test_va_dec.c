/* SPDX-License-Identifier: MIT */
/* VAAPI decode harness: VCPTIME bundle in, visible raw frames out.
 * Usage: test_va_dec <bundle.vctime> <out.raw> [--async N] [--surfaces N]
 * Exit 0 iff every record decodes and syncs; prints frame log. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>

#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_dec_hevc.h>
#include <va/va_dec_vp9.h>
#ifdef TEST_DMABUF_GPU
#include "test_dmabuf_gpu.h"
#endif

#define DIE(...) do { fprintf(stderr, "FAIL: " __VA_ARGS__); \
                      fprintf(stderr, "\n"); return 1; } while (0)
#define CHECKST(fn, st) do { if ((st) != VA_STATUS_SUCCESS) { \
    fprintf(stderr, "FAIL: %s -> %d\n", fn, st); return 1; } } while (0)

static uint32_t rd32(FILE *fp, int *ok)
{
    uint32_t v;
    if (fread(&v, 1, 4, fp) != 4) { *ok = 0; return 0; }
    return v;
}

int main(int argc, char **argv)
{
    const char *bundle_path, *out_path;
    int async = 0, nsurf_req = 0, main10 = 0, pace_ms = 0;
    FILE *bfp, *ofp;
    int ok = 1, drmfd;
    VADisplay dpy;
    VAStatus st;
    uint32_t codec, count, bw, bh;
    char magic[8];
    uint8_t **aus;
    uint32_t *ausz;
    unsigned int i, n;
    VAProfile profile;
    size_t pic_param_size;
    VAConfigID config = VA_INVALID_ID;
    VAContextID context = VA_INVALID_ID;
    VASurfaceID *surfs = NULL;
    int nsurf;
    int frames_ok = 0;

    if (argc < 3) {
        fprintf(stderr,
                "usage: %s <bundle.vctime> <out.raw> [--async N] "
                "[--surfaces N]\n", argv[0]);
        return 2;
    }
    bundle_path = argv[1];
    out_path = argv[2];
    for (i = 3; i < (unsigned int)argc; i++) {
        if (!strcmp(argv[i], "--async") && i + 1 < (unsigned int)argc)
            async = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--surfaces") && i + 1 < (unsigned int)argc)
            nsurf_req = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--main10"))
            main10 = 1;
        else if (!strcmp(argv[i], "--pace") && i + 1 < (unsigned int)argc)
            pace_ms = atoi(argv[++i]);
    }
    bfp = fopen(bundle_path, "rb");
    if (!bfp) DIE("open bundle %s", bundle_path);
    if (fread(magic, 1, 8, bfp) != 8 || memcmp(magic, "VCPTIME1", 8))
        DIE("bad bundle magic");
    codec = rd32(bfp, &ok); count = rd32(bfp, &ok);
    bw = rd32(bfp, &ok); bh = rd32(bfp, &ok);
    if (!ok || !count || count > 4096) DIE("bad bundle header");
    aus = calloc(count, sizeof(*aus));
    ausz = calloc(count, sizeof(*ausz));
    for (i = 0; i < count; i++) {
        uint32_t sz;
        uint64_t pts;
        sz = rd32(bfp, &ok);
        if (fread(&pts, 1, 8, bfp) != 8) ok = 0;
        (void)pts;
        if (!ok || !sz || sz > 4 * 1024 * 1024) DIE("bad record %u", i);
        ausz[i] = sz;
        aus[i] = malloc(sz);
        if (!aus[i] || fread(aus[i], 1, sz, bfp) != sz)
            DIE("short record %u", i);
    }
    fclose(bfp);

    switch (codec) {
    case 0x34363248: /* 'H264' */
        profile = VAProfileH264High; pic_param_size = sizeof(VAPictureParameterBufferH264);
        break;
    case 0x43564548: /* 'HEVC' */
        profile = main10 ? VAProfileHEVCMain10 : VAProfileHEVCMain;
        pic_param_size = sizeof(VAPictureParameterBufferHEVC);
        break;
    case 0x30395056: /* 'VP90' */
        profile = VAProfileVP9Profile0; pic_param_size = sizeof(VADecPictureParameterBufferVP9);
        break;
    case 0x3247504d: /* 'MPG2' */
        profile = VAProfileMPEG2Main; pic_param_size = sizeof(VAPictureParameterBufferMPEG2);
        break;
    default:
        DIE("unsupported bundle codec 0x%08x", codec);
    }

    drmfd = open("/dev/dri/renderD128", O_RDWR);
    if (drmfd < 0) DIE("open render node");
    dpy = vaGetDisplayDRM(drmfd);
    if (!dpy) DIE("vaGetDisplayDRM");
    {
        int major = 0, minor = 0;
        st = vaInitialize(dpy, &major, &minor);
        CHECKST("vaInitialize", st);
        fprintf(stderr, "va-api %d.%d\n", major, minor);
    }
    fprintf(stderr, "vendor: %s\n", vaQueryVendorString(dpy));

    st = vaCreateConfig(dpy, profile, VAEntrypointVLD, NULL, 0, &config);
    CHECKST("vaCreateConfig", st);

    nsurf = nsurf_req > 0 ? nsurf_req : (int)count;
    if (nsurf > (int)count) nsurf = (int)count;
    if (nsurf < 2) nsurf = 2;
    surfs = calloc((size_t)nsurf, sizeof(*surfs));
    st = vaCreateSurfaces(dpy, VA_RT_FORMAT_YUV420, bw, bh, surfs,
                          (unsigned int)nsurf, NULL, 0);
    CHECKST("vaCreateSurfaces", st);
    st = vaCreateContext(dpy, config, (int)bw, (int)bh, VA_PROGRESSIVE,
                         surfs, nsurf, &context);
    CHECKST("vaCreateContext", st);

    ofp = fopen(out_path, "wb");
    if (!ofp) DIE("open output %s", out_path);

    int inflight = 0;
    for (n = 0; n < count; n++) {
        VASurfaceID target = surfs[n % (unsigned int)nsurf];
        VABufferID picp = VA_INVALID_ID, sliced = VA_INVALID_ID;
        void *pp;
        st = vaBeginPicture(dpy, context, target);
        CHECKST("vaBeginPicture", st);
        st = vaCreateBuffer(dpy, context, VAPictureParameterBufferType,
                            (unsigned int)pic_param_size, 1, NULL, &picp);
        CHECKST("vaCreateBuffer(pic)", st);
        st = vaMapBuffer(dpy, picp, &pp);
        CHECKST("vaMapBuffer(pic)", st);
        memset(pp, 0, pic_param_size);
        st = vaUnmapBuffer(dpy, picp);
        CHECKST("vaUnmapBuffer(pic)", st);
        st = vaRenderPicture(dpy, context, &picp, 1);
        CHECKST("vaRenderPicture(pic)", st);
        st = vaDestroyBuffer(dpy, picp);
        CHECKST("vaDestroyBuffer(pic)", st);
        st = vaCreateBuffer(dpy, context, VASliceDataBufferType,
                            ausz[n], 1, aus[n], &sliced);
        CHECKST("vaCreateBuffer(slice)", st);
        st = vaRenderPicture(dpy, context, &sliced, 1);
        CHECKST("vaRenderPicture(slice)", st);
        st = vaDestroyBuffer(dpy, sliced);
        CHECKST("vaDestroyBuffer(slice)", st);
        st = vaEndPicture(dpy, context);
        CHECKST("vaEndPicture", st);
        if (pace_ms > 0) {
            struct timespec ts = { pace_ms / 1000,
                                   (pace_ms % 1000) * 1000000L };
            nanosleep(&ts, NULL);
        }
        inflight++;
        if (!async || inflight > async) {
            VASurfaceID sync_target =
                surfs[(n - (unsigned int)(inflight - 1)) %
                      (unsigned int)nsurf];
            st = vaSyncSurface(dpy, sync_target);
            CHECKST("vaSyncSurface", st);
            /* dump */
            {
                VAImage img;
                void *mp;
                VABufferID imgb = VA_INVALID_ID;
                (void)imgb;
                st = vaDeriveImage(dpy, sync_target, &img);
                CHECKST("vaDeriveImage", st);
                st = vaMapBuffer(dpy, img.buf, &mp);
                CHECKST("vaMapBuffer(img)", st);
#ifdef TEST_DMABUF_GPU
                if (gpu_check(dpy, sync_target, &img, mp, bw, bh, main10))
                    DIE("GPU exact comparison frame %u", n);
#endif
                /* visible crop: width bw, drop stride/padding */
                {
                    uint8_t *base = mp;
                    unsigned int y;
                    size_t row = (size_t)bw * (main10 ? 2 : 1);
                    for (y = 0; y < bh; y++)
                        fwrite(base + img.offsets[0] +
                               (size_t)y * img.pitches[0], 1, row, ofp);
                    for (y = 0; y < bh / 2; y++)
                        fwrite(base + img.offsets[1] +
                               (size_t)y * img.pitches[1], 1, row, ofp);
                }
                st = vaUnmapBuffer(dpy, img.buf);
                CHECKST("vaUnmapBuffer(img)", st);
                st = vaDestroyImage(dpy, img.image_id);
                CHECKST("vaDestroyImage", st);
            }
            frames_ok++;
            inflight--;
            fprintf(stderr, "frame %u ok (surface %u)\n", n,
                    sync_target);
        }
    }
    /* Drain the rest in order, then destroy (exercises drain path
     * when async left pictures in flight). */
    for (i = 0; i < (unsigned int)inflight; i++) {
        VASurfaceID sync_target =
            surfs[(count - (unsigned int)inflight + i) %
                  (unsigned int)nsurf];
        VAImage img;
        void *mp;
        st = vaSyncSurface(dpy, sync_target);
        CHECKST("vaSyncSurface(drain)", st);
        st = vaDeriveImage(dpy, sync_target, &img);
        CHECKST("vaDeriveImage(drain)", st);
        st = vaMapBuffer(dpy, img.buf, &mp);
        CHECKST("vaMapBuffer(img,drain)", st);
#ifdef TEST_DMABUF_GPU
        if (gpu_check(dpy, sync_target, &img, mp, bw, bh, main10))
            DIE("GPU exact comparison drain %u", i);
#endif
        {
            uint8_t *base = mp;
            unsigned int y;
            size_t row = (size_t)bw * (main10 ? 2 : 1);
            for (y = 0; y < bh; y++)
                fwrite(base + img.offsets[0] + (size_t)y * img.pitches[0],
                       1, row, ofp);
            for (y = 0; y < bh / 2; y++)
                fwrite(base + img.offsets[1] + (size_t)y * img.pitches[1],
                       1, row, ofp);
        }
        st = vaUnmapBuffer(dpy, img.buf);
        CHECKST("vaUnmapBuffer(img,drain)", st);
        st = vaDestroyImage(dpy, img.image_id);
        CHECKST("vaDestroyImage(drain)", st);
        frames_ok++;
        fprintf(stderr, "frame drain %u ok\n", count - inflight + i);
    }
    fclose(ofp);
#ifdef TEST_DMABUF_GPU
    gpu_fini();
#endif
    st = vaDestroyContext(dpy, context);
    CHECKST("vaDestroyContext", st);
    st = vaDestroySurfaces(dpy, surfs, nsurf);
    CHECKST("vaDestroySurfaces", st);
    st = vaDestroyConfig(dpy, config);
    CHECKST("vaDestroyConfig", st);
    st = vaTerminate(dpy);
    CHECKST("vaTerminate", st);
    close(drmfd);
    printf("HARNESS-OK frames=%d/%u\n", frames_ok, count);
    return frames_ok == (int)count ? 0 : 1;
}
