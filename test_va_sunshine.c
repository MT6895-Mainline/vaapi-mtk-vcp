/* SPDX-License-Identifier: MIT */
/* Simulates the FFmpeg/Sunshine vaapi encode loop against mtk_vcp:
 * a pool of NV12 surfaces exported WRITE_ONLY|SEPARATE_LAYERS (the flags
 * libavcodec uses), pixels written into the exported dma-buf the way the
 * EGL converter would, then encoded with an in-flight depth like the
 * ffmpeg async pipeline. Measures sustained fps.
 * Usage: test_va_sunshine --w W --h H --frames N [--fps F] [--bitrate B]
 *                         [--depth D] [--pattern P] [--raw FILE] <out.264>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>

#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_drmcommon.h>
#include <va/va_enc_h264.h>

#define DIE(...) do { fprintf(stderr, "FAIL: " __VA_ARGS__); fprintf(stderr, "\n"); return 1; } while (0)
#define CHECKST(fn, st) do { if ((st) != VA_STATUS_SUCCESS) { \
    fprintf(stderr, "FAIL: %s -> %d\n", fn, (int)(st)); return 1; } } while (0)
#define MAXD 8

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + ts.tv_nsec / 1000;
}

struct slot {
    VASurfaceID surf;
    void *map;
    size_t size, uvoff;
    int fd, pitch;
};

static void fill(struct slot *sl, int w, int h, int i, int pattern)
{
    int x, y;

    if (pattern == 4) {
        /* Position ramp: every 4 KiB block filled with its block number,
         * so the decoded chroma values reveal where the firmware read. */
        unsigned char *base = sl->map;
        size_t blk;
        for (blk = 0; blk * 4096 < sl->size; blk++) {
            size_t len = sl->size - blk * 4096;
            if (len > 4096)
                len = 4096;
            memset(base + blk * 4096, (int)(blk & 0xff), len);
        }
        return;
    }
    if (pattern == 3) {
        /* Marker experiment: distinct UV pairs at the three candidate
         * chroma offsets; Y rows constant 42. */
        size_t offA = (size_t)2460 * 1080;   /* visible w * h */
        size_t offB = (size_t)2464 * 1080;   /* align16 w * h */
        size_t offC = sl->uvoff;             /* reported (align64 w * h) */
        unsigned char *base = sl->map;
        for (y = 0; y < h; y++)
            memset(base + (size_t)y * sl->pitch, 42, (size_t)w);
        memset(base + offA, 111, (size_t)w * 10);
        for (x = 0; x < w * 5; x += 2) { base[offA + x] = 111; base[offA + x + 1] = 222; }
        for (x = 0; x < w * 5; x += 2) { base[offB + x] = 150; base[offB + x + 1] = 50; }
        for (x = 0; x < w * 5; x += 2) { base[offC + x] = 200; base[offC + x + 1] = 100; }
        return;
    }
    if (pattern == 2) {
        for (y = 0; y < h; y++)
            memset((uint8_t *)sl->map + (size_t)y * sl->pitch, 42,
                   (size_t)w);
        for (y = 0; y < h / 2; y++) {
            uint8_t *row = (uint8_t *)sl->map + sl->uvoff +
                           (size_t)y * sl->pitch;
            for (x = 0; x < w; x += 2) {
                row[x] = 200;
                row[x + 1] = 100;
            }
        }
        return;
    }
    if (pattern == 1) {
        for (y = 0; y < h; y++)
            memset((uint8_t *)sl->map + (size_t)y * sl->pitch,
                   (uint8_t)(y & 0xff), (size_t)w);
        for (y = 0; y < h / 2; y++) {
            uint8_t *row = (uint8_t *)sl->map + sl->uvoff +
                           (size_t)y * sl->pitch;
            memset(row, 200, (size_t)w / 2);
            memset(row + w / 2, 100, (size_t)(w - w / 2));
        }
        return;
    }
    for (y = 0; y < h; y++) {
        uint8_t *row = (uint8_t *)sl->map + (size_t)y * sl->pitch;
        for (x = 0; x < w; x++)
            row[x] = (uint8_t)((x * 3 + y * 5 + i * 7) & 0xff);
    }
    for (y = 0; y < h / 2; y++) {
        uint8_t *row = (uint8_t *)sl->map + sl->uvoff +
                       (size_t)y * sl->pitch;
        for (x = 0; x < w; x += 2) {
            row[x] = (uint8_t)(128 + ((x + i * 13) & 0x7f));
            row[x + 1] = (uint8_t)(128 - ((y + i * 5) & 0x7f));
        }
    }
}

int main(int argc, char **argv)
{
    int w = 2460, h = 1080, ctx_w = 0, ctx_h = 0;
    int nframes = 300, fps = 60, bitrate = 20000000;
    int pattern = 0, depth = 1, uv_at_ctx_h = 0;
    const char *out_path = NULL, *raw_path = NULL;
    int i, d, major = 0, minor = 0, drmfd;
    VADisplay dpy;
    VAConfigID config;
    VAContextID context;
    struct slot slots[MAXD];
    VABufferID *coded;
    FILE *ofp, *rfp = NULL;
    uint64_t t0, t1;
    size_t total = 0;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--w") && i + 1 < argc) w = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--h") && i + 1 < argc) h = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--ctx-w") && i + 1 < argc) ctx_w = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--ctx-h") && i + 1 < argc) ctx_h = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--frames") && i + 1 < argc) nframes = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--fps") && i + 1 < argc) fps = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--bitrate") && i + 1 < argc) bitrate = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pattern") && i + 1 < argc) pattern = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--uv-at-ctx-h")) uv_at_ctx_h = 1;
        else if (!strcmp(argv[i], "--depth") && i + 1 < argc) depth = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--raw") && i + 1 < argc) raw_path = argv[++i];
        else out_path = argv[i];
    }
    if (!out_path || depth < 1 || depth > MAXD)
        DIE("usage: test_va_sunshine --w W --h H --frames N <out.264>");
    if (!ctx_w) ctx_w = w;
    if (!ctx_h) ctx_h = h;
    ofp = fopen(out_path, "wb");
    if (!ofp) DIE("open out");
    if (raw_path) {
        rfp = fopen(raw_path, "wb");
        if (!rfp) DIE("open raw");
    }
    coded = calloc((size_t)nframes, sizeof(VABufferID));
    if (!coded) DIE("oom");

    drmfd = open("/dev/dri/renderD128", O_RDWR);
    if (drmfd < 0) DIE("open render node");
    dpy = vaGetDisplayDRM(drmfd);
    if (!dpy) DIE("vaGetDisplayDRM");
    CHECKST("vaInitialize", vaInitialize(dpy, &major, &minor));
    CHECKST("vaCreateConfig", vaCreateConfig(dpy, VAProfileH264High,
              VAEntrypointEncSlice, NULL, 0, &config));
    /* FFmpeg creates the encode context with no pre-bound targets; the
     * hwframe pool allocates surfaces later and they stay UNBOUND until
     * first use, which is what makes the WRITE_ONLY export possible. */
    CHECKST("vaCreateContext", vaCreateContext(dpy, config, ctx_w, ctx_h,
              VA_PROGRESSIVE, NULL, 0, &context));

    for (d = 0; d < depth; d++) {
        VADRMPRIMESurfaceDescriptor prime;
        off_t osz;

        memset(&slots[d], 0, sizeof(slots[d]));
        slots[d].fd = -1;
        CHECKST("vaCreateSurfaces", vaCreateSurfaces(dpy,
                  VA_RT_FORMAT_YUV420, w, h, &slots[d].surf, 1, NULL, 0));
        memset(&prime, 0, sizeof(prime));
        CHECKST("vaExportSurfaceHandle",
                vaExportSurfaceHandle(dpy, slots[d].surf,
                    VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
                    VA_EXPORT_SURFACE_WRITE_ONLY |
                    VA_EXPORT_SURFACE_SEPARATE_LAYERS, &prime));
        slots[d].fd = prime.objects[0].fd;
        osz = lseek(slots[d].fd, 0, SEEK_END);
        if (osz <= 0) DIE("export size slot %d", d);
        slots[d].size = (size_t)osz;
        slots[d].map = mmap(NULL, slots[d].size, PROT_READ | PROT_WRITE,
                            MAP_SHARED, slots[d].fd, 0);
        if (slots[d].map == MAP_FAILED) DIE("mmap slot %d", d);
        slots[d].pitch = (int)prime.layers[0].pitch[0];
        slots[d].uvoff = prime.layers[1].offset[0];
        /* Diagnostic only: emulate a producer that writes chroma after
         * the padded context rows instead of the exported visible rows. */
        if (uv_at_ctx_h) {
            size_t offset = (size_t)slots[d].pitch * (size_t)ctx_h;
            if (offset + (size_t)slots[d].pitch * (size_t)(h / 2) > slots[d].size)
                DIE("padded UV offset exceeds export object");
            slots[d].uvoff = offset;
        }
        fprintf(stderr, "slot %d: fd=%d size=%zu pitch=%d uvoff=%zu\n",
                d, slots[d].fd, slots[d].size, slots[d].pitch,
                slots[d].uvoff);
    }

    t0 = now_us();
    for (i = 0; i < nframes; i++) {
        struct slot *sl = &slots[i % depth];
        VAEncSequenceParameterBufferH264 s;
        VAEncPictureParameterBufferH264 p;
        VAEncSliceParameterBufferH264 sp;
        VAEncMiscParameterBuffer *m;
        VAEncMiscParameterRateControl *rc;
        VAEncMiscParameterFrameRate *fr;
        uint8_t mbuf[sizeof(*m) + sizeof(*rc)];
        uint8_t fbuf[sizeof(*m) + sizeof(*fr)];
        VABufferID seq, rcbuf, frbuf, pic, slice;

        fill(sl, w, h, i, pattern);
        if (rfp && i == 0) {
            fwrite(sl->map, 1, (size_t)sl->pitch * h, rfp);
            fwrite((uint8_t *)sl->map + sl->uvoff, 1,
                   (size_t)sl->pitch * (h / 2), rfp);
        }
        memset(&s, 0, sizeof(s));
        s.intra_period = 30;
        s.intra_idr_period = 30;
        s.bits_per_second = (uint32_t)bitrate;
        CHECKST("seq", vaCreateBuffer(dpy, context,
                  VAEncSequenceParameterBufferType, sizeof(s), 1, &s, &seq));
        memset(mbuf, 0, sizeof(mbuf));
        m = (VAEncMiscParameterBuffer *)mbuf;
        m->type = VAEncMiscParameterTypeRateControl;
        rc = (VAEncMiscParameterRateControl *)m->data;
        rc->bits_per_second = (uint32_t)bitrate;
        rc->target_percentage = 100;
        CHECKST("misc rc", vaCreateBuffer(dpy, context,
                  VAEncMiscParameterBufferType, sizeof(mbuf), 1, mbuf,
                  &rcbuf));
        memset(fbuf, 0, sizeof(fbuf));
        m = (VAEncMiscParameterBuffer *)fbuf;
        m->type = VAEncMiscParameterTypeFrameRate;
        fr = (VAEncMiscParameterFrameRate *)m->data;
        fr->framerate = (uint32_t)fps;
        CHECKST("misc fps", vaCreateBuffer(dpy, context,
                  VAEncMiscParameterBufferType, sizeof(fbuf), 1, fbuf,
                  &frbuf));
        CHECKST("coded", vaCreateBuffer(dpy, context, VAEncCodedBufferType,
                  1 << 21, 1, NULL, &coded[i]));
        memset(&p, 0, sizeof(p));
        p.coded_buf = coded[i];
        CHECKST("pic", vaCreateBuffer(dpy, context,
                  VAEncPictureParameterBufferType, sizeof(p), 1, &p, &pic));
        memset(&sp, 0, sizeof(sp));
        sp.slice_type = (i % 30) ? 0 : 2;
        sp.num_macroblocks = (uint32_t)(w / 16) * (uint32_t)(h / 16);
        if (!sp.slice_type)
            sp.idr_pic_id = 1;
        CHECKST("slice", vaCreateBuffer(dpy, context,
                  VAEncSliceParameterBufferType, sizeof(sp), 1, &sp,
                  &slice));
        CHECKST("vaBeginPicture", vaBeginPicture(dpy, context, sl->surf));
        CHECKST("render seq", vaRenderPicture(dpy, context, &seq, 1));
        CHECKST("render rc", vaRenderPicture(dpy, context, &rcbuf, 1));
        CHECKST("render fps", vaRenderPicture(dpy, context, &frbuf, 1));
        CHECKST("render pic", vaRenderPicture(dpy, context, &pic, 1));
        CHECKST("render slice", vaRenderPicture(dpy, context, &slice, 1));
        CHECKST("vaEndPicture", vaEndPicture(dpy, context));
        vaDestroyBuffer(dpy, seq);
        vaDestroyBuffer(dpy, rcbuf);
        vaDestroyBuffer(dpy, frbuf);
        vaDestroyBuffer(dpy, pic);
        vaDestroyBuffer(dpy, slice);

        /* Retire the frame that left the pipeline depth ago. */
        if (i >= depth) {
            VACodedBufferSegment *seg;

            CHECKST("sync", vaSyncBuffer(dpy, coded[i - depth], 0));
            CHECKST("map", vaMapBuffer(dpy, coded[i - depth],
                      (void **)&seg));
            if (!seg || !seg->buf || !seg->size)
                DIE("empty coded frame %d", i - depth);
            fwrite(seg->buf, 1, seg->size, ofp);
            total += seg->size;
            vaUnmapBuffer(dpy, coded[i - depth]);
            vaDestroyBuffer(dpy, coded[i - depth]);
        }
    }
    for (i = nframes - depth; i < nframes; i++) {
        VACodedBufferSegment *seg;

        CHECKST("sync tail", vaSyncBuffer(dpy, coded[i], 0));
        CHECKST("map tail", vaMapBuffer(dpy, coded[i], (void **)&seg));
        if (!seg || !seg->buf || !seg->size)
            DIE("empty coded frame %d", i);
        fwrite(seg->buf, 1, seg->size, ofp);
        total += seg->size;
        vaUnmapBuffer(dpy, coded[i]);
        vaDestroyBuffer(dpy, coded[i]);
    }
    t1 = now_us();
    fclose(ofp);
    if (rfp) fclose(rfp);
    fprintf(stderr, "SUNSHINE-SIM %dx%d depth=%d frames=%d wall=%.3fs -> %.1f fps bytes=%zu\n",
            w, h, depth, nframes, (t1 - t0) / 1e6,
            nframes * 1e6 / (double)(t1 - t0), total);
    return 0;
}
