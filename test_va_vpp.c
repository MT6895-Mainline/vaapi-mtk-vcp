/* SPDX-License-Identifier: MIT */
/* VAAPI VPP harness: PRIME import + CPU convert/scale (never boots VCP).
 * Exit 0 iff import/VPP/readback verify against an independent
 * double-precision reference. Usage: test_va_vpp */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <libdrm/drm_fourcc.h>

#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_vpp.h>
#include <va/va_drmcommon.h>

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

/* Independent double-precision BT.601 full->limited reference. */
static void ref601(int r, int g, int b, int *y, int *u, int *v)
{
    double dd;
    dd = 16.0 + (65.481 * r + 128.553 * g + 24.966 * b) / 255.0;
    *y = (int)(dd + 0.5);
    dd = 128.0 + (-37.797 * r - 74.203 * g + 112.0 * b) / 255.0;
    *u = (int)(dd + 0.5);
    dd = 128.0 + (112.0 * r - 93.786 * g - 18.214 * b) / 255.0;
    *v = (int)(dd + 0.5);
}

static void ref709(int r, int g, int b, int *y, int *u, int *v)
{
    double dd;
    dd = 16.0 + (46.559 * r + 156.629 * g + 15.812 * b) / 255.0;
    *y = (int)(dd + 0.5);
    dd = 128.0 + (-25.664 * r - 86.336 * g + 112.0 * b) / 255.0;
    *u = (int)(dd + 0.5);
    dd = 128.0 + (112.0 * r - 101.730 * g - 10.270 * b) / 255.0;
    *v = (int)(dd + 0.5);
}

/* Wrap-free 320x240 pattern shared by fill and reference. */
static void pat3(int x, int y, int *r, int *g, int *b)
{
    *r = x / 2;               /* 0..159 */
    *g = y;                   /* 0..239 */
    *b = (*r + *g > 255) ? 255 : (*r + *g);
}

static int adiff(int a, int b)
{
    int d = a - b;
    return d < 0 ? -d : d;
}

static VADisplay dpy;
static int nfail = 0;

static void run_vpp(VAContextID ctx, VASurfaceID in, VASurfaceID target,
                    int std, int *status)
{
    VAProcPipelineParameterBuffer params;
    VABufferID buf;
    VAStatus st;
    memset(&params, 0, sizeof(params));
    params.surface = in;
    params.surface_region = NULL;
    params.surface_color_standard = VAProcColorStandardNone;
    params.output_region = NULL;
    params.output_color_standard = std ? VAProcColorStandardBT709 :
                                         VAProcColorStandardBT601;
    params.pipeline_flags = 0;
    params.filter_flags = VA_FILTER_SCALING_FAST;
    params.filters = NULL;
    params.num_filters = 0;
    st = vaBeginPicture(dpy, ctx, target);
    if (st != VA_STATUS_SUCCESS) {
        fprintf(stderr, "FAIL: BeginPicture -> %d\n", st);
        *status = 1;
        return;
    }
    st = vaCreateBuffer(dpy, ctx, VAProcPipelineParameterBufferType,
                        sizeof(params), 1, &params, &buf);
    if (st != VA_STATUS_SUCCESS) {
        fprintf(stderr, "FAIL: CreateBuffer(pipeline) -> %d\n", st);
        *status = 1;
        return;
    }
    st = vaRenderPicture(dpy, ctx, &buf, 1);
    if (st != VA_STATUS_SUCCESS)
        fprintf(stderr, "FAIL: RenderPicture -> %d\n", st);
    st = vaEndPicture(dpy, ctx);
    if (st != VA_STATUS_SUCCESS)
        fprintf(stderr, "FAIL: EndPicture -> %d\n", st);
    vaDestroyBuffer(dpy, buf);
    if (st != VA_STATUS_SUCCESS)
        *status = 1;
}

static uint8_t *read_target(VASurfaceID target, int w, int h, int *stride)
{
    VAImage img;
    void *mp;
    VAStatus st;
    uint8_t *out;
    st = vaDeriveImage(dpy, target, &img);
    if (st != VA_STATUS_SUCCESS) {
        fprintf(stderr, "FAIL: DeriveImage -> %d\n", st);
        return NULL;
    }
    st = vaMapBuffer(dpy, img.buf, &mp);
    if (st != VA_STATUS_SUCCESS) {
        fprintf(stderr, "FAIL: MapBuffer -> %d\n", st);
        vaDestroyImage(dpy, img.image_id);
        return NULL;
    }
    out = malloc((size_t)img.width * img.height * 3 / 2);
    if (!out) {
        vaUnmapBuffer(dpy, img.buf);
        vaDestroyImage(dpy, img.image_id);
        return NULL;
    }
    memcpy(out, mp, (size_t)img.width * img.height * 3 / 2);
    *stride = img.pitches[0];
    vaUnmapBuffer(dpy, img.buf);
    vaDestroyImage(dpy, img.image_id);
    (void)w; (void)h;
    return out;
}

int main(void)
{
    int drmfd, major = 0, minor = 0;
    VAStatus st;
    VAConfigID cfg;
    VAContextID vpp320, vpp160;
    int status = 0;

    drmfd = open("/dev/dri/renderD128", O_RDWR);
    if (drmfd < 0)
        DIE("open render node");
    dpy = vaGetDisplayDRM(drmfd);
    if (!dpy)
        DIE("vaGetDisplayDRM");
    st = vaInitialize(dpy, &major, &minor);
    CHECKST("vaInitialize", st);
    st = vaCreateConfig(dpy, VAProfileNone, VAEntrypointVideoProc,
                        NULL, 0, &cfg);
    CHECKST("vaCreateConfig(vpp)", st);

    /* ---- test 1: XR24 -> NV12 same size (601) ---- */
    {
        int w = 320, h = 240, stride = w * 4;
        size_t size = (size_t)stride * h;
        int fd = memfd_new("vpp-xr24", size);
        uint8_t *m;
        VADRMPRIMESurfaceDescriptor desc;
        VASurfaceAttrib attribs[2];
        VASurfaceID in = VA_INVALID_ID, target = VA_INVALID_ID;
        uint8_t *got;
        int got_stride, x, y, bad = 0;
        if (fd < 0)
            DIE("memfd");
        m = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (m == MAP_FAILED)
            DIE("mmap pattern");
        for (y = 0; y < h; y++)
            for (x = 0; x < w; x++) {
                uint8_t *p = m + (size_t)y * stride + (size_t)x * 4;
                p[0] = (uint8_t)((x * 3 + y) & 0xff);       /* B */
                p[1] = (uint8_t)((y * 5 + 11) & 0xff);      /* G */
                p[2] = (uint8_t)((x * 7 + y * 3) & 0xff);   /* R */
                p[3] = 0;
            }
        munmap(m, size);
        memset(&desc, 0, sizeof(desc));
        desc.width = (uint32_t)w;
        desc.height = (uint32_t)h;
        desc.num_objects = 1;
        desc.objects[0].fd = fd;
        desc.objects[0].size = (uint32_t)size;
        desc.objects[0].drm_format_modifier = DRM_FORMAT_MOD_LINEAR;
        desc.num_layers = 1;
        desc.layers[0].drm_format = DRM_FORMAT_XRGB8888;
        desc.layers[0].num_planes = 1;
        desc.layers[0].object_index[0] = 0;
        desc.layers[0].offset[0] = 0;
        desc.layers[0].pitch[0] = (uint32_t)stride;
        attribs[0].type = VASurfaceAttribMemoryType;
        attribs[0].flags = VA_SURFACE_ATTRIB_SETTABLE;
        attribs[0].value.type = VAGenericValueTypeInteger;
        attribs[0].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2;
        attribs[1].type = VASurfaceAttribExternalBufferDescriptor;
        attribs[1].flags = VA_SURFACE_ATTRIB_SETTABLE;
        attribs[1].value.type = VAGenericValueTypePointer;
        attribs[1].value.value.p = &desc;
        st = vaCreateSurfaces(dpy, VA_RT_FORMAT_RGB32, w, h, &in, 1,
                              attribs, 2);
        /* The import dups the fd; the original may go away. */
        close(fd);
        CHECKST("import XR24", st);
        st = vaCreateSurfaces(dpy, VA_RT_FORMAT_YUV420, w, h, &target,
                              1, NULL, 0);
        CHECKST("target surface", st);
        st = vaCreateContext(dpy, cfg, w, h, VA_PROGRESSIVE, NULL, 0,
                             &vpp320);
        CHECKST("vaCreateContext(vpp320)", st);
        run_vpp(vpp320, in, target, 0, &status);
        got = read_target(target, w, h, &got_stride);
        if (!got)
            DIE("readback t1");
        for (y = 0; y < h && bad < 8; y++)
            for (x = 0; x < w && bad < 8; x++) {
                int r = (x * 7 + y * 3) & 0xff;
                int g = (y * 5 + 11) & 0xff;
                int b = (x * 3 + y) & 0xff;
                int ey, eu, ev;
                ref601(r, g, b, &ey, &eu, &ev);
                if (adiff(got[y * got_stride + x], ey) > 2)
                    bad++;
            }
        for (y = 0; y < h / 2 && bad < 8; y++)
            for (x = 0; x < w && bad < 8; x += 2) {
                int r = (x * 7 + (y * 2) * 3) & 0xff;
                int g = ((y * 2) * 5 + 11) & 0xff;
                int b = (x * 3 + y * 2) & 0xff;
                int ey, eu, ev;
                uint8_t *uv = got + got_stride * h;
                ref601(r, g, b, &ey, &eu, &ev);
                if (adiff(uv[y * got_stride + x], eu) > 2 ||
                    adiff(uv[y * got_stride + x + 1], ev) > 2)
                    bad++;
            }
        printf("t1 xr24->nv12 same-size: %s\n",
               bad ? "MISMATCH" : "exact-ish");
        if (bad)
            nfail++;
        free(got);
        vaDestroySurfaces(dpy, &in, 1);
        vaDestroySurfaces(dpy, &target, 1);
    }

    /* ---- test 2: NV12 -> NV12 same size must be bit-exact ---- */
    {
        int w = 320, h = 240, stride = 320;
        size_t size = (size_t)stride * h * 3 / 2;
        int fd = memfd_new("vpp-nv12", size);
        uint8_t *m, *pat;
        VADRMPRIMESurfaceDescriptor desc;
        VASurfaceAttrib attribs[2];
        VASurfaceID in = VA_INVALID_ID, target = VA_INVALID_ID;
        uint8_t *got;
        int got_stride, bad = 0, i;
        if (fd < 0)
            DIE("memfd");
        pat = malloc(size);
        if (!pat)
            DIE("oom");
        for (i = 0; (size_t)i < size; i++)
            pat[i] = (uint8_t)((i * 2654435761u >> 16) & 0xff);
        m = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (m == MAP_FAILED)
            DIE("mmap pattern");
        memcpy(m, pat, size);
        munmap(m, size);
        memset(&desc, 0, sizeof(desc));
        desc.width = (uint32_t)w;
        desc.height = (uint32_t)h;
        desc.num_objects = 1;
        desc.objects[0].fd = fd;
        desc.objects[0].size = (uint32_t)size;
        desc.objects[0].drm_format_modifier = DRM_FORMAT_MOD_LINEAR;
        desc.num_layers = 1;
        desc.layers[0].drm_format = DRM_FORMAT_NV12;
        desc.layers[0].num_planes = 2;
        desc.layers[0].object_index[0] = 0;
        desc.layers[0].object_index[1] = 0;
        desc.layers[0].offset[0] = 0;
        desc.layers[0].pitch[0] = (uint32_t)stride;
        desc.layers[0].offset[1] = (uint32_t)(stride * h);
        desc.layers[0].pitch[1] = (uint32_t)stride;
        attribs[0].type = VASurfaceAttribMemoryType;
        attribs[0].flags = VA_SURFACE_ATTRIB_SETTABLE;
        attribs[0].value.type = VAGenericValueTypeInteger;
        attribs[0].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2;
        attribs[1].type = VASurfaceAttribExternalBufferDescriptor;
        attribs[1].flags = VA_SURFACE_ATTRIB_SETTABLE;
        attribs[1].value.type = VAGenericValueTypePointer;
        attribs[1].value.value.p = &desc;
        st = vaCreateSurfaces(dpy, VA_RT_FORMAT_YUV420, w, h, &in, 1,
                              attribs, 2);
        close(fd);
        CHECKST("import NV12", st);
        st = vaCreateSurfaces(dpy, VA_RT_FORMAT_YUV420, w, h, &target,
                              1, NULL, 0);
        CHECKST("target surface", st);
        run_vpp(vpp320, in, target, 0, &status);
        got = read_target(target, w, h, &got_stride);
        if (!got)
            DIE("readback t2");
        if (got_stride != stride ||
            memcmp(got, pat, size) != 0)
            bad = 1;
        printf("t2 nv12->nv12 same-size: %s\n",
               bad ? "MISMATCH" : "EXACT");
        if (bad)
            nfail++;
        free(pat);
        free(got);
        vaDestroySurfaces(dpy, &in, 1);
        vaDestroySurfaces(dpy, &target, 1);
    }

    /* ---- test 3: XR24 320x240 -> 160x120 (709, scaled) ---- */
    {
        int sw = 320, sh = 240, dw = 160, dh = 120, stride = sw * 4;
        size_t size = (size_t)stride * sh;
        int fd = memfd_new("vpp-scale", size);
        uint8_t *m;
        VADRMPRIMESurfaceDescriptor desc;
        VASurfaceAttrib attribs[2];
        VASurfaceID in = VA_INVALID_ID, target = VA_INVALID_ID;
        uint8_t *got;
        int got_stride, x, y, bad = 0;
        if (fd < 0)
            DIE("memfd");
        m = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (m == MAP_FAILED)
            DIE("mmap pattern");
        for (y = 0; y < sh; y++)
            for (x = 0; x < sw; x++) {
                uint8_t *p = m + (size_t)y * stride + (size_t)x * 4;
                int r, g, b;
                pat3(x, y, &r, &g, &b);
                p[0] = (uint8_t)b;
                p[1] = (uint8_t)g;
                p[2] = (uint8_t)r;
                p[3] = 0;
            }
        munmap(m, size);
        memset(&desc, 0, sizeof(desc));
        desc.width = (uint32_t)sw;
        desc.height = (uint32_t)sh;
        desc.num_objects = 1;
        desc.objects[0].fd = fd;
        desc.objects[0].size = (uint32_t)size;
        desc.objects[0].drm_format_modifier = DRM_FORMAT_MOD_LINEAR;
        desc.num_layers = 1;
        desc.layers[0].drm_format = DRM_FORMAT_XRGB8888;
        desc.layers[0].num_planes = 1;
        desc.layers[0].object_index[0] = 0;
        desc.layers[0].offset[0] = 0;
        desc.layers[0].pitch[0] = (uint32_t)stride;
        attribs[0].type = VASurfaceAttribMemoryType;
        attribs[0].flags = VA_SURFACE_ATTRIB_SETTABLE;
        attribs[0].value.type = VAGenericValueTypeInteger;
        attribs[0].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2;
        attribs[1].type = VASurfaceAttribExternalBufferDescriptor;
        attribs[1].flags = VA_SURFACE_ATTRIB_SETTABLE;
        attribs[1].value.type = VAGenericValueTypePointer;
        attribs[1].value.value.p = &desc;
        st = vaCreateSurfaces(dpy, VA_RT_FORMAT_RGB32, sw, sh, &in, 1,
                              attribs, 2);
        close(fd);
        CHECKST("import XR24 scale", st);
        st = vaCreateSurfaces(dpy, VA_RT_FORMAT_YUV420, dw, dh, &target,
                              1, NULL, 0);
        CHECKST("target surface", st);
        st = vaCreateContext(dpy, cfg, dw, dh, VA_PROGRESSIVE, NULL, 0,
                             &vpp160);
        CHECKST("vaCreateContext(vpp160)", st);
        run_vpp(vpp160, in, target, 1, &status);
        got = read_target(target, dw, dh, &got_stride);
        if (!got)
            DIE("readback t3");
        /* Bilinear ideal in double precision, then 709 convert. */
        for (y = 0; y < dh && bad < 8; y++)
            for (x = 0; x < dw && bad < 8; x++) {
                double fx = ((2 * x + 1) * sw - dw) / (double)(2 * dw);
                double fy = ((2 * y + 1) * sh - dh) / (double)(2 * dh);
                int x0 = (int)fx, y0 = (int)fy;
                double ax = fx - x0, ay = fy - y0;
                int r00, g00, b00, r01, g01, b01;
                int r10, g10, b10, r11, g11, b11;
                double r, g, b;
                int ey, eu, ev;
                if (x0 < 0) { x0 = 0; ax = 0; }
                if (y0 < 0) { y0 = 0; ay = 0; }
                if (x0 >= sw - 1) { x0 = sw - 1; ax = 0; }
                if (y0 >= sh - 1) { y0 = sh - 1; ay = 0; }
                {
                    int xa = x0 + 1 < sw ? x0 + 1 : x0;
                    int ya = y0 + 1 < sh ? y0 + 1 : y0;
                    pat3(x0, y0, &r00, &g00, &b00);
                    pat3(xa, y0, &r01, &g01, &b01);
                    pat3(x0, ya, &r10, &g10, &b10);
                    pat3(xa, ya, &r11, &g11, &b11);
                }
                r = (r00 * (1 - ax) + r01 * ax) * (1 - ay) +
                    (r10 * (1 - ax) + r11 * ax) * ay;
                g = (g00 * (1 - ax) + g01 * ax) * (1 - ay) +
                    (g10 * (1 - ax) + g11 * ax) * ay;
                b = (b00 * (1 - ax) + b01 * ax) * (1 - ay) +
                    (b10 * (1 - ax) + b11 * ax) * ay;
                ref709((int)(r + 0.5), (int)(g + 0.5), (int)(b + 0.5),
                       &ey, &eu, &ev);
                if (adiff(got[y * got_stride + x], ey) > 4) {
                    if (bad == 0)
                        fprintf(stderr,
                                "t3 first diff at %d,%d: got %d want %d\n",
                                x, y, got[y * got_stride + x], ey);
                    bad++;
                }
                (void)eu; (void)ev;
            }
        printf("t3 xr24 320x240->160x120 luma: %s\n",
               bad ? "MISMATCH" : "close");
        if (bad)
            nfail++;
        free(got);
        vaDestroySurfaces(dpy, &in, 1);
        vaDestroySurfaces(dpy, &target, 1);
        vaDestroyContext(dpy, vpp160);
    }

    /* ---- test 4: refusals ---- */
    {
        VADRMPRIMESurfaceDescriptor desc;
        VASurfaceAttrib attribs[2];
        VASurfaceID in = VA_INVALID_ID;
        VAProcPipelineParameterBuffer params;
        VABufferID buf;
        int fd = memfd_new("vpp-neg", 64 * 64 * 4);
        if (fd < 0)
            DIE("memfd");
        memset(&desc, 0, sizeof(desc));
        desc.width = 64;
        desc.height = 64;
        desc.num_objects = 1;
        desc.objects[0].fd = fd;
        desc.objects[0].size = 64 * 64 * 4;
        /* Bogus non-linear modifier must be refused. */
        desc.objects[0].drm_format_modifier = 0x0100000000000001ULL;
        desc.num_layers = 1;
        desc.layers[0].drm_format = DRM_FORMAT_XRGB8888;
        desc.layers[0].num_planes = 1;
        desc.layers[0].pitch[0] = 64 * 4;
        attribs[0].type = VASurfaceAttribMemoryType;
        attribs[0].flags = VA_SURFACE_ATTRIB_SETTABLE;
        attribs[0].value.type = VAGenericValueTypeInteger;
        attribs[0].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2;
        attribs[1].type = VASurfaceAttribExternalBufferDescriptor;
        attribs[1].flags = VA_SURFACE_ATTRIB_SETTABLE;
        attribs[1].value.type = VAGenericValueTypePointer;
        attribs[1].value.value.p = &desc;
        st = vaCreateSurfaces(dpy, VA_RT_FORMAT_RGB32, 64, 64, &in, 1,
                              attribs, 2);
        close(fd);
        if (st == VA_STATUS_SUCCESS) {
            fprintf(stderr, "FAIL: tiled import accepted\n");
            nfail++;
            vaDestroySurfaces(dpy, &in, 1);
        } else {
            printf("t4a tiled import refused (%d): ok\n", st);
        }
        /* Rotation must be refused (identity only): use a valid
         * surface so the rotation gate itself is exercised. */
        memset(&params, 0, sizeof(params));
        {
            int vfd = memfd_new("vpp-rot", 64 * 64 * 4);
            uint8_t *vm;
            VASurfaceID vsurf = VA_INVALID_ID;
            if (vfd >= 0) {
                vm = mmap(NULL, 64 * 64 * 4, PROT_READ | PROT_WRITE,
                          MAP_SHARED, vfd, 0);
                if (vm != MAP_FAILED) {
                    memset(vm, 0x80, 64 * 64 * 4);
                    munmap(vm, 64 * 64 * 4);
                }
                memset(&desc, 0, sizeof(desc));
                desc.width = 64;
                desc.height = 64;
                desc.num_objects = 1;
                desc.objects[0].fd = vfd;
                desc.objects[0].size = 64 * 64 * 4;
                desc.objects[0].drm_format_modifier =
                    DRM_FORMAT_MOD_LINEAR;
                desc.num_layers = 1;
                desc.layers[0].drm_format = DRM_FORMAT_XRGB8888;
                desc.layers[0].num_planes = 1;
                desc.layers[0].pitch[0] = 64 * 4;
                if (vaCreateSurfaces(dpy, VA_RT_FORMAT_RGB32, 64, 64,
                                     &vsurf, 1, attribs,
                                     2) == VA_STATUS_SUCCESS)
                    params.surface = vsurf;
                close(vfd);
            }
        }
        if (params.surface == VA_INVALID_ID) {
            fprintf(stderr, "FAIL: t4 valid import\n");
            nfail++;
        }
        params.rotation_state = VA_ROTATION_90;
        st = vaCreateBuffer(dpy, vpp320, VAProcPipelineParameterBufferType,
                            sizeof(params), 1, &params, &buf);
        if (st != VA_STATUS_SUCCESS) {
            fprintf(stderr, "FAIL: pipeline buffer create -> %d\n", st);
            nfail++;
        } else {
            VASurfaceID tgt = VA_INVALID_ID;
            if (vaCreateSurfaces(dpy, VA_RT_FORMAT_YUV420, 320, 240,
                                 &tgt, 1, NULL, 0) != VA_STATUS_SUCCESS ||
                vaBeginPicture(dpy, vpp320, tgt) != VA_STATUS_SUCCESS) {
                fprintf(stderr, "FAIL: t4 setup\n");
                nfail++;
            } else {
                st = vaRenderPicture(dpy, vpp320, &buf, 1);
                if (st == VA_STATUS_SUCCESS) {
                    fprintf(stderr, "FAIL: rotation accepted\n");
                    nfail++;
                } else {
                    printf("t4b rotation refused (%d): ok\n", st);
                }
            }
            vaDestroyBuffer(dpy, buf);
            if (tgt != VA_INVALID_ID)
                vaDestroySurfaces(dpy, &tgt, 1);
        }
    }

    vaDestroyContext(dpy, vpp320);
    vaDestroyConfig(dpy, cfg);
    vaTerminate(dpy);
    close(drmfd);
    printf(nfail || status ? "VPP RESULT: FAIL\n" : "VPP RESULT: PASS\n");
    return (nfail || status) ? 1 : 0;
}
