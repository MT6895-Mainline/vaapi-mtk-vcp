/* SPDX-License-Identifier: MIT */
/* PRIME export + GetImage validation: decode one frame, export it as a
 * dmabuf, import the fd into the DRM device, and cross-check GetImage
 * bytes against DeriveImage bytes.
 * Usage: test_va_prime <bundle.vctime>
 * Exit 0 iff all checks pass. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_drmcommon.h>

/* Use the UAPI header: PRIME_FD_TO_HANDLE is 0x2e, not a driver ioctl. */
#include <libdrm/drm.h>
#include <libdrm/drm_fourcc.h>

#define DIE(...) do { fprintf(stderr, "FAIL: " __VA_ARGS__); \
                      fprintf(stderr, "\n"); return 1; } while (0)
#define CHECKST(fn, st) do { if ((st) != VA_STATUS_SUCCESS) { \
    fprintf(stderr, "FAIL: %s -> %d\n", fn, st); return 1; } } while (0)

static int probe_export(VADisplay dpy, unsigned int rt)
{
    VASurfaceID surface;
    VADRMPRIMESurfaceDescriptor a, b;
    struct stat sa, sb;
    int ten = rt == VA_RT_FORMAT_YUV420_10;
    CHECKST("probe create", vaCreateSurfaces(dpy, rt, 128, 128, &surface, 1, NULL, 0));
    CHECKST("probe export", vaExportSurfaceHandle(dpy, surface,
        VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
        VA_EXPORT_SURFACE_READ_ONLY | VA_EXPORT_SURFACE_SEPARATE_LAYERS, &a));
    CHECKST("probe repeat", vaExportSurfaceHandle(dpy, surface,
        VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
        VA_EXPORT_SURFACE_READ_ONLY | VA_EXPORT_SURFACE_COMPOSED_LAYERS, &b));
    if (a.num_objects != 1 || a.num_layers != 2 || b.num_layers != 1 ||
        a.layers[0].drm_format != (ten ? DRM_FORMAT_R16 : DRM_FORMAT_R8) ||
        a.layers[1].drm_format != (ten ? DRM_FORMAT_GR1616 : DRM_FORMAT_GR88) ||
        a.layers[1].offset[0] != b.layers[0].offset[1] ||
        a.objects[0].size < (unsigned)(128 * 128 * 3 / (ten ? 1 : 2)) ||
        !(fcntl(a.objects[0].fd, F_GETFD) & FD_CLOEXEC) ||
        fstat(a.objects[0].fd, &sa) || fstat(b.objects[0].fd, &sb) ||
        sa.st_ino != sb.st_ino) DIE("probe descriptor/identity");
    close(b.objects[0].fd);
    CHECKST("probe destroy", vaDestroySurfaces(dpy, &surface, 1));
    /* Export keeps the zeroed allocation alive after queue/surface destruction. */
    unsigned char *map = mmap(NULL, a.objects[0].size, PROT_READ, MAP_SHARED,
                              a.objects[0].fd, 0);
    if (map == MAP_FAILED) DIE("probe lifetime mmap");
    for (unsigned int i = 0; i < a.objects[0].size; i++)
        if (map[i]) DIE("uninitialized exported byte %u", i);
    munmap(map, a.objects[0].size); close(a.objects[0].fd);
    printf("probe: %s separate/composed, identity, zero-fill and lifetime OK\n",
           ten ? "P010" : "NV12");
    return 0;
}

int main(int argc, char **argv)
{
    const char *bundle_path = argv[1];
    FILE *bfp;
    char magic[8];
    uint32_t codec, count, bw, bh;
    uint8_t *au0 = NULL;
    uint32_t au0sz = 0;
    int drmfd, major = 0, minor = 0;
    VADisplay dpy;
    VAStatus st;
    VAProfile profile;
    VAConfigID config;
    VAContextID context;
    VASurfaceID surf, surfs[2];
    VABufferID picp, sliced;
    void *pp;
    VAImage dimg, cimg;
    void *dmap, *cmap;
    int i, ok = 1;

    if (argc < 2) {
        fprintf(stderr, "usage: %s <bundle.vctime>\n", argv[0]);
        return 2;
    }
    bfp = fopen(bundle_path, "rb");
    if (!bfp) DIE("open bundle");
    if (fread(magic, 1, 8, bfp) != 8 || memcmp(magic, "VCPTIME1", 8))
        DIE("bad magic");
    if (fread(&codec, 4, 1, bfp) != 1 || fread(&count, 4, 1, bfp) != 1 ||
        fread(&bw, 4, 1, bfp) != 1 || fread(&bh, 4, 1, bfp) != 1)
        DIE("bad header");
    if (!count) DIE("empty bundle");
    {
        uint32_t sz;
        uint64_t pts;
        if (fread(&sz, 4, 1, bfp) != 1 || fread(&pts, 8, 1, bfp) != 1)
            DIE("bad record");
        (void)pts;
        au0 = malloc(sz);
        au0sz = sz;
        if (!au0 || fread(au0, 1, sz, bfp) != sz) DIE("short record");
    }
    fclose(bfp);
    profile = (codec == 0x34363248) ? VAProfileH264High : VAProfileHEVCMain;

    drmfd = open("/dev/dri/renderD128", O_RDWR);
    if (drmfd < 0) DIE("open render node");
    dpy = vaGetDisplayDRM(drmfd);
    if (!dpy) DIE("vaGetDisplayDRM");
    st = vaInitialize(dpy, &major, &minor);
    CHECKST("vaInitialize", st);
    if (probe_export(dpy, VA_RT_FORMAT_YUV420) ||
        probe_export(dpy, VA_RT_FORMAT_YUV420_10)) return 1;
    st = vaCreateConfig(dpy, profile, VAEntrypointVLD, NULL, 0, &config);
    CHECKST("vaCreateConfig", st);
    st = vaCreateSurfaces(dpy, VA_RT_FORMAT_YUV420, bw, bh, surfs, 2,
                          NULL, 0);
    CHECKST("vaCreateSurfaces", st);
    surf = surfs[0];
    st = vaCreateContext(dpy, config, bw, bh, VA_PROGRESSIVE, surfs, 2,
                         &context);
    CHECKST("vaCreateContext", st);
    st = vaBeginPicture(dpy, context, surf);
    CHECKST("vaBeginPicture", st);
    st = vaCreateBuffer(dpy, context, VAPictureParameterBufferType, 2048,
                        1, NULL, &picp);
    CHECKST("vaCreateBuffer(pic)", st);
    st = vaMapBuffer(dpy, picp, &pp);
    CHECKST("vaMapBuffer(pic)", st);
    memset(pp, 0, 2048);
    st = vaUnmapBuffer(dpy, picp);
    CHECKST("vaUnmapBuffer(pic)", st);
    st = vaRenderPicture(dpy, context, &picp, 1);
    CHECKST("vaRenderPicture(pic)", st);
    st = vaDestroyBuffer(dpy, picp);
    CHECKST("vaDestroyBuffer(pic)", st);
    st = vaCreateBuffer(dpy, context, VASliceDataBufferType, au0sz, 1,
                        au0, &sliced);
    CHECKST("vaCreateBuffer(slice)", st);
    st = vaRenderPicture(dpy, context, &sliced, 1);
    CHECKST("vaRenderPicture(slice)", st);
    st = vaDestroyBuffer(dpy, sliced);
    CHECKST("vaDestroyBuffer(slice)", st);
    st = vaEndPicture(dpy, context);
    CHECKST("vaEndPicture", st);
    st = vaSyncSurface(dpy, surf);
    CHECKST("vaSyncSurface", st);

    setvbuf(stdout, NULL, _IONBF, 0);
    /* 1. DeriveImage baseline. */
    st = vaDeriveImage(dpy, surf, &dimg);
    CHECKST("vaDeriveImage", st);
    st = vaMapBuffer(dpy, dimg.buf, &dmap);
    CHECKST("vaMapBuffer(derive)", st);

    /* 2. PRIME export. */
    {
        VADRMPRIMESurfaceDescriptor desc;
        st = vaExportSurfaceHandle(dpy, surf,
                                   VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
                                   VA_EXPORT_SURFACE_READ_ONLY, &desc);
        CHECKST("vaExportSurfaceHandle", st);
        printf("prime: fourcc=0x%08x %ux%u objs=%u layers=%u fd=%d\n",
               desc.fourcc, desc.width, desc.height, desc.num_objects,
               desc.num_layers, desc.objects[0].fd);
        if (desc.num_objects != 1 || desc.num_layers != 1 ||
            desc.objects[0].size < dimg.data_size ||
            desc.layers[0].num_planes != 2 ||
            desc.width != bw || desc.height != bh) {
            fprintf(stderr, "FAIL: prime geometry mismatch\n");
            ok = 0;
        }
        if (desc.layers[0].pitch[0] != (uint32_t)dimg.pitches[0] ||
            desc.layers[0].offset[1] != (uint32_t)dimg.offsets[1]) {
            fprintf(stderr, "FAIL: prime pitch/offset vs derive "
                    "(%u/%d, %u/%d)\n", desc.layers[0].pitch[0],
                    dimg.pitches[0], desc.layers[0].offset[1],
                    dimg.offsets[1]);
            ok = 0;
        }
        /* Import the dmabuf into DRM devices: proves the fd is a
         * real, live dma-buf. renderD128 is panthor (GPU); card0 is
         * mediatek-drm (display). */
        {
            struct drm_prime_handle ph;
            int card0;
            memset(&ph, 0, sizeof(ph));
            ph.fd = desc.objects[0].fd;
            if (ioctl(drmfd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &ph) < 0) {
                perror("FAIL: renderD128 FD_TO_HANDLE");
                ok = 0;
            } else {
                printf("prime: renderD128 imported handle %u\n",
                       ph.handle);
                struct drm_gem_close gc = { .handle = ph.handle };
                if (ioctl(drmfd, DRM_IOCTL_GEM_CLOSE, &gc) < 0)
                    DIE("GPU GEM_CLOSE");
            }
            card0 = open("/dev/dri/card0", O_RDWR);
            if (card0 < 0) {
                perror("FAIL: open card0");
                ok = 0;
            } else {
                memset(&ph, 0, sizeof(ph));
                ph.fd = desc.objects[0].fd;
                if (ioctl(card0, DRM_IOCTL_PRIME_FD_TO_HANDLE, &ph) < 0) {
                    /* A failed import must fail this regression test. */
                    perror("FAIL: card0 FD_TO_HANDLE");
                    ok = 0;
                } else {
                    printf("prime: card0 imported handle %u\n",
                           ph.handle);
                }
                close(card0);
            }
        }
        close(desc.objects[0].fd);
        /* PRIME v1 has a different descriptor; reject rather than overwrite it. */
        st = vaExportSurfaceHandle(dpy, surf, VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME,
                                   VA_EXPORT_SURFACE_READ_ONLY, &desc);
        if (st != VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE)
            DIE("PRIME v1 must be rejected");
        st = vaExportSurfaceHandle(dpy, surf, VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
            VA_EXPORT_SURFACE_SEPARATE_LAYERS | VA_EXPORT_SURFACE_COMPOSED_LAYERS, &desc);
        if (st != VA_STATUS_ERROR_INVALID_PARAMETER)
            DIE("conflicting layer flags must be rejected");
        /* WRITE_ONLY must be refused (decode output is read-only). */
        st = vaExportSurfaceHandle(dpy, surf,
                                   VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
                                   VA_EXPORT_SURFACE_WRITE_ONLY, &desc);
        if (st == VA_STATUS_SUCCESS) {
            fprintf(stderr, "FAIL: WRITE_ONLY export accepted\n");
            close(desc.objects[0].fd);
            ok = 0;
        } else {
            printf("prime: WRITE_ONLY refused (%d) ok\n", st);
        }
    }

    /* 3. GetImage full frame must equal DeriveImage bytes. */
    {
        VAImageFormat fmt;
        fmt.fourcc = VA_FOURCC_NV12;
        fmt.byte_order = VA_LSB_FIRST;
        fmt.bits_per_pixel = 12;
        st = vaCreateImage(dpy, &fmt, bw, bh, &cimg);
        CHECKST("vaCreateImage", st);
        st = vaGetImage(dpy, surf, 0, 0, bw, bh, cimg.image_id);
        CHECKST("vaGetImage", st);
        st = vaMapBuffer(dpy, cimg.buf, &cmap);
        CHECKST("vaMapBuffer(get)", st);
        for (i = 0; i < 2; i++) {
            unsigned int y;
            uint8_t *sp = (uint8_t *)dmap +
                          (i ? dimg.offsets[1] : dimg.offsets[0]);
            uint8_t *dp = (uint8_t *)cmap +
                          (i ? cimg.offsets[1] : cimg.offsets[0]);
            unsigned int rows = i ? bh / 2 : bh;
            for (y = 0; y < rows; y++) {
                if (memcmp(sp + (size_t)y * dimg.pitches[i],
                           dp + (size_t)y * cimg.pitches[i], bw) != 0) {
                    fprintf(stderr, "FAIL: GetImage != Derive "
                            "plane %d row %u\n", i, y);
                    ok = 0;
                    y = rows;
                    i = 2;
                }
            }
        }
        if (ok)
            printf("getimage: full-frame bytes match derive\n");
        st = vaUnmapBuffer(dpy, cimg.buf);
        CHECKST("vaUnmapBuffer(get)", st);
        st = vaDestroyImage(dpy, cimg.image_id);
        CHECKST("vaDestroyImage(get)", st);
    }
    st = vaUnmapBuffer(dpy, dimg.buf);
    CHECKST("vaUnmapBuffer(derive)", st);
    st = vaDestroyImage(dpy, dimg.image_id);
    CHECKST("vaDestroyImage(derive)", st);
    st = vaDestroyContext(dpy, context);
    CHECKST("vaDestroyContext", st);
    st = vaDestroySurfaces(dpy, surfs, 2);
    CHECKST("vaDestroySurfaces", st);
    st = vaDestroyConfig(dpy, config);
    CHECKST("vaDestroyConfig", st);
    st = vaTerminate(dpy);
    CHECKST("vaTerminate", st);
    close(drmfd);
    free(au0);
    printf(ok ? "PRIME-OK\n" : "PRIME-FAIL\n");
    return ok ? 0 : 1;
}
