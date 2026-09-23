/* SPDX-License-Identifier: MIT */
/* Test-only GPU oracle: persistent EGL imports, no CPU texture uploads.
 * Compare shader readback to the decoded CPU view on every reuse. */
#include <sys/stat.h>
#include <va/va_drmcommon.h>
#include <gbm.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>
#include <GLES2/gl2ext.h>

struct gpu_import {
    ino_t ino;
    unsigned int offset, format, width, height;
    EGLImageKHR image;
    GLuint texture;
};
static struct {
    int fd, count, hits;
    struct gbm_device *gbm;
    EGLDisplay display;
    EGLContext context;
    GLuint program, fbo, output;
    PFNEGLCREATEIMAGEKHRPROC create_image;
    PFNEGLDESTROYIMAGEKHRPROC destroy_image;
    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC image_texture;
    struct gpu_import imports[128];
} gpu;

static GLuint gpu_shader(GLenum type, const char *source)
{
    GLuint shader = glCreateShader(type);
    GLint ok;
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetShaderInfoLog(shader, sizeof(log), NULL, log);
        fprintf(stderr, "shader: %s\n", log);
        return 0;
    }
    return shader;
}

static int gpu_init(void)
{
    EGLint major, minor, n;
    EGLConfig config;
    const EGLint attrs[] = { EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
                            EGL_SURFACE_TYPE, EGL_WINDOW_BIT, EGL_NONE };
    const EGLint ctxattrs[] = { EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE };
    const char *vs = "#version 300 es\n"
        "void main(){vec2 p=vec2((gl_VertexID<<1)&2,gl_VertexID&2);"
        "gl_Position=vec4(p*2.0-1.0,0.0,1.0);}";
    const char *fs = "#version 300 es\nprecision highp float;"
        "precision highp int;uniform highp sampler2D src;"
        "uniform bool ten;out vec4 color;void main(){"
        "vec2 v=texelFetch(src,ivec2(gl_FragCoord.xy),0).rg;"
        "if(ten){uvec2 s=uvec2(round(v*65535.0));"
        "color=vec4(s.x&255u,s.x>>8,s.y&255u,s.y>>8)/255.0;}"
        "else color=vec4(v,0.0,1.0);}";
    GLuint v, f;
    GLint linked;
    gpu.fd = open("/dev/dri/renderD128", O_RDWR | O_CLOEXEC);
    if (gpu.fd < 0 || !(gpu.gbm = gbm_create_device(gpu.fd))) return -1;
    gpu.display = eglGetPlatformDisplay(EGL_PLATFORM_GBM_KHR, gpu.gbm, NULL);
    if (!eglInitialize(gpu.display, &major, &minor) ||
        !eglBindAPI(EGL_OPENGL_ES_API) ||
        !eglChooseConfig(gpu.display, attrs, &config, 1, &n) || !n) return -1;
    gpu.context = eglCreateContext(gpu.display, config, EGL_NO_CONTEXT, ctxattrs);
    if (gpu.context == EGL_NO_CONTEXT ||
        !eglMakeCurrent(gpu.display, EGL_NO_SURFACE, EGL_NO_SURFACE, gpu.context))
        return -1;
    fprintf(stderr, "GPU renderer: %s\n", glGetString(GL_RENDERER));
    if (strstr((const char *)glGetString(GL_RENDERER), "llvmpipe")) return -1;
    gpu.create_image = (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
    gpu.destroy_image = (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
    gpu.image_texture = (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)
        eglGetProcAddress("glEGLImageTargetTexture2DOES");
    if (!gpu.create_image || !gpu.destroy_image || !gpu.image_texture) return -1;
    v = gpu_shader(GL_VERTEX_SHADER, vs); f = gpu_shader(GL_FRAGMENT_SHADER, fs);
    if (!v || !f) return -1;
    gpu.program = glCreateProgram();
    glAttachShader(gpu.program, v); glAttachShader(gpu.program, f);
    glLinkProgram(gpu.program); glDeleteShader(v); glDeleteShader(f);
    glGetProgramiv(gpu.program, GL_LINK_STATUS, &linked);
    if (!linked) return -1;
    glUseProgram(gpu.program);
    glUniform1i(glGetUniformLocation(gpu.program, "src"), 0);
    glGenFramebuffers(1, &gpu.fbo); glGenTextures(1, &gpu.output);
    return 0;
}

static int gpu_check(VADisplay dpy, VASurfaceID surface, const VAImage *img,
                     const void *cpu, unsigned int w, unsigned int h, int ten)
{
    VADRMPRIMESurfaceDescriptor desc;
    struct stat sb;
    int rc = -1;
    if (!gpu.program && gpu_init()) return -1;
    if (vaExportSurfaceHandle(dpy, surface, VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
            VA_EXPORT_SURFACE_READ_ONLY | VA_EXPORT_SURFACE_SEPARATE_LAYERS,
            &desc) != VA_STATUS_SUCCESS) return -1;
    if (desc.num_objects != 1 || desc.num_layers != 2 || !desc.objects[0].size ||
        fstat(desc.objects[0].fd, &sb)) goto out;
    for (unsigned int plane = 0; plane < 2; plane++) {
        unsigned int pw = plane ? w / 2 : w, ph = plane ? h / 2 : h;
        struct gpu_import *entry = NULL;
        for (int i = 0; i < gpu.count; i++) {
            struct gpu_import *e = &gpu.imports[i];
            if (e->ino == sb.st_ino && e->offset == desc.layers[plane].offset[0] &&
                e->format == desc.layers[plane].drm_format && e->width == pw &&
                e->height == ph) { entry = e; gpu.hits++; break; }
        }
        if (!entry) {
            EGLint a[] = { EGL_WIDTH, (EGLint)pw, EGL_HEIGHT, (EGLint)ph,
                EGL_LINUX_DRM_FOURCC_EXT, desc.layers[plane].drm_format,
                EGL_DMA_BUF_PLANE0_FD_EXT, desc.objects[0].fd,
                EGL_DMA_BUF_PLANE0_OFFSET_EXT, desc.layers[plane].offset[0],
                EGL_DMA_BUF_PLANE0_PITCH_EXT, desc.layers[plane].pitch[0], EGL_NONE };
            if (gpu.count == 128) goto out;
            entry = &gpu.imports[gpu.count++];
            entry->ino = sb.st_ino; entry->offset = desc.layers[plane].offset[0];
            entry->format = desc.layers[plane].drm_format;
            entry->width = pw; entry->height = ph;
            entry->image = gpu.create_image(gpu.display, EGL_NO_CONTEXT,
                                            EGL_LINUX_DMA_BUF_EXT, NULL, a);
            if (entry->image == EGL_NO_IMAGE_KHR) {
                fprintf(stderr, "EGL import failed: 0x%x\n", eglGetError()); goto out;
            }
            glGenTextures(1, &entry->texture);
            glBindTexture(GL_TEXTURE_2D, entry->texture);
            gpu.image_texture(GL_TEXTURE_2D, entry->image);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        }
        glBindTexture(GL_TEXTURE_2D, gpu.output);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, pw, ph, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, NULL);
        glBindFramebuffer(GL_FRAMEBUFFER, gpu.fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, gpu.output, 0);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) goto out;
        glViewport(0, 0, pw, ph);
        glBindTexture(GL_TEXTURE_2D, entry->texture);
        glUniform1i(glGetUniformLocation(gpu.program, "ten"), ten);
        glDisable(GL_DITHER);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        unsigned char *pixels = malloc((size_t)pw * ph * 4);
        if (!pixels) goto out;
        glReadPixels(0, 0, pw, ph, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
        if (glGetError() != GL_NO_ERROR) { free(pixels); goto out; }
        unsigned int comps = plane ? 2 : 1, bpp = ten ? 2 : 1;
        size_t errors = 0;
        for (unsigned int y = 0; y < ph; y++) {
            const unsigned char *row = (const unsigned char *)cpu + img->offsets[plane] +
                                       (size_t)y * img->pitches[plane];
            for (unsigned int x = 0; x < pw; x++)
                for (unsigned int c = 0; c < comps * bpp; c++)
                    if (pixels[((size_t)y * pw + x) * 4 + c] != row[x * comps * bpp + c])
                        errors++;
        }
        free(pixels);
        if (errors) { fprintf(stderr, "GPU mismatch plane=%u bytes=%zu\n", plane, errors); goto out; }
    }
    rc = 0;
out:
    close(desc.objects[0].fd);
    return rc;
}

static void gpu_fini(void)
{
    if (!gpu.program) return;
    fprintf(stderr, "GPU-EXACT imports=%d cache_hits=%d\n", gpu.count, gpu.hits);
    for (int i = 0; i < gpu.count; i++) {
        glDeleteTextures(1, &gpu.imports[i].texture);
        gpu.destroy_image(gpu.display, gpu.imports[i].image);
    }
    glDeleteTextures(1, &gpu.output); glDeleteFramebuffers(1, &gpu.fbo);
    glDeleteProgram(gpu.program);
    eglMakeCurrent(gpu.display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(gpu.display, gpu.context); eglTerminate(gpu.display);
    gbm_device_destroy(gpu.gbm); close(gpu.fd);
}
