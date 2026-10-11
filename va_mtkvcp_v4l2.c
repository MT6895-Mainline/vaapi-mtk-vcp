/* SPDX-License-Identifier: MIT */
/* Small V4L2 M2M helpers. Every ioctl retries on EINTR; errors are
 * returned as negative errno. No logging here (caller decides). */
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/videodev2.h>
#include <linux/dma-heap.h>
#include <linux/dma-buf.h>

#include "va_mtkvcp.h"

/*
 * Is this fd a dma-buf rather than an ordinary file? A memfd or a plain
 * file can be mmapped and copied from, but the V4L2 DMABUF memory type
 * needs a real exporter behind the fd. DMA_BUF_IOCTL_SYNC is only
 * implemented by dma-buf, so it doubles as the test.
 */
int mtkvcp_fd_is_dmabuf(int fd)
{
    struct dma_buf_sync sync = { .flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW };
    struct dma_buf_sync end = { .flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW };
    int r;

    if (fd < 0)
        return 0;
    r = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
    if (r == 0)
        ioctl(fd, DMA_BUF_IOCTL_SYNC, &end);
    return r == 0;
}

/*
 * Bracket a CPU read of a dma-buf that somebody else (a GPU, the firmware)
 * wrote: START invalidates stale cache lines and waits for the writer's
 * fences where the exporter implements them, END releases the access.
 * Returns 0 when the sync ioctls were accepted.
 */
int mtkvcp_dmabuf_cpu_read(int fd)
{
    struct dma_buf_sync sync = { .flags = DMA_BUF_SYNC_START |
                                        DMA_BUF_SYNC_READ };
    int r;

    if (fd < 0)
        return -EINVAL;
    r = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
    if (r < 0)
        return -errno;
    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    if (ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync) < 0)
        return -errno;
    return 0;
}

/*
 * Bracket a CPU write into a dma-buf that a device will read next. The
 * START side tells the exporter to expect CPU stores; the END side makes
 * them visible to the device.
 */
int mtkvcp_dmabuf_cpu_write(int fd)
{
    struct dma_buf_sync sync = { .flags = DMA_BUF_SYNC_START |
                                        DMA_BUF_SYNC_WRITE };
    int r;

    if (fd < 0)
        return -EINVAL;
    r = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
    if (r < 0)
        return -errno;
    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
    if (ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync) < 0)
        return -errno;
    return 0;
}

/*
 * Allocate a dma-buf from the system heap. The capture path needs one to
 * hand the encoder a buffer it can DMA from directly; without a heap the
 * OUTPUT queue would have to stay in MMAP memory and every frame would be
 * copied through userspace first.
 */
int mtkvcp_dma_heap_alloc(size_t size)
{
    struct dma_heap_allocation_data d = {
        .len = size,
        .fd_flags = O_RDWR | O_CLOEXEC,
    };
    int h = open("/dev/dma_heap/default_cma_region", O_RDWR | O_CLOEXEC);
    if (h < 0)
        h = open("/dev/dma_heap/system", O_RDWR | O_CLOEXEC);
    if (h < 0)
        return -errno;
    if (ioctl(h, DMA_HEAP_IOCTL_ALLOC, &d) < 0) {
        int e = errno;
        close(h);
        return -e;
    }
    close(h);
    return (int)d.fd;
}

/*
 * Allocate from the generic system heap. Bounce buffers only need to be
 * readable by the GPU, and the CMA region is tiny (32 MB on xaga) and
 * shared with the display and VCP paths, so never take those from it.
 */
int mtkvcp_dma_heap_alloc_system(size_t size)
{
    struct dma_heap_allocation_data d = {
        .len = size,
        .fd_flags = O_RDWR | O_CLOEXEC,
    };
    int h = open("/dev/dma_heap/system", O_RDWR | O_CLOEXEC);

    if (h < 0)
        return -errno;
    if (ioctl(h, DMA_HEAP_IOCTL_ALLOC, &d) < 0) {
        int e = errno;

        close(h);
        return -e;
    }
    close(h);
    return (int)d.fd;
}

int mtkvcp_xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do {
        r = ioctl(fd, req, arg);
    } while (r < 0 && errno == EINTR);
    return r < 0 ? -errno : 0;
}

/* ---- codec node discovery ---- */

/* "video170" -> 170; anything else -> -1. */
static int node_index(const char *name)
{
    const char *p = name;
    int v = 0;

    if (strncmp(p, "video", 5))
        return -1;
    p += 5;
    if (!*p)
        return -1;
    for (; *p; p++) {
        if (*p < '0' || *p > '9')
            return -1;
        v = v * 10 + (*p - '0');
    }
    return v;
}

/* Descending by video index: the codec nodes probe after every other V4L2
 * driver, so they sit at the top and are reached first. */
static int cmp_video_desc(const void *a, const void *b)
{
    return node_index(*(const char *const *)b) -
           node_index(*(const char *const *)a);
}

/* Does /sys/class/video4linux/<entry> report this video_device name? */
static int node_name_is(const char *sysfs_entry, const char *want)
{
    char path[PATH_MAX], name[128];
    ssize_t n;
    int fd;

    snprintf(path, sizeof(path), "%s/name", sysfs_entry);
    fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0)
        return 0;
    n = read(fd, name, sizeof(name) - 1);
    close(fd);
    if (n <= 0)
        return 0;
    name[n] = '\0';
    while (n > 0 && (name[n - 1] == '\n' || name[n - 1] == '\r'))
        name[--n] = '\0';
    return strcmp(name, want) == 0;
}

/*
 * Resolve one codec node. An explicit override always wins; otherwise match
 * the video_device name against sysfs, walking the nodes from the highest
 * index down. The index is not stable: enabling another V4L2 driver (mtk-isp
 * registers ~170 camera nodes) renumbers every /dev/videoN after it, so the
 * name is the only reliable key and the reverse walk makes the lookup cheap.
 */
static void resolve_node(char *dst, size_t dstsz, const char *env,
                         const char *name, const char *fallback)
{
    const char *v = getenv(env);
    glob_t g;

    if (v && *v) {
        snprintf(dst, dstsz, "%s", v);
        return;
    }
    if (glob("/sys/class/video4linux/video*", 0, NULL, &g) == 0) {
        size_t i;

        qsort(g.gl_pathv, g.gl_pathc, sizeof(*g.gl_pathv), cmp_video_desc);
        for (i = 0; i < g.gl_pathc; i++) {
            const char *base = strrchr(g.gl_pathv[i], '/');

            if (!base || !node_name_is(g.gl_pathv[i], name))
                continue;
            snprintf(dst, dstsz, "/dev/%s", base + 1);
            break;
        }
        globfree(&g);
    }
    if (!dst[0])
        snprintf(dst, dstsz, "%s", fallback);
}

static pthread_mutex_t node_lock = PTHREAD_MUTEX_INITIALIZER;
static char node_path[2][64];
static int nodes_done;

static void resolve_nodes(void)
{
    if (nodes_done)
        return;
    pthread_mutex_lock(&node_lock);
    if (!nodes_done) {
        resolve_node(node_path[0], sizeof(node_path[0]),
                     "MTK_VCP_VA_DEC_NODE", "mtk-vcp-dec", MTKVCP_DEC_NODE);
        resolve_node(node_path[1], sizeof(node_path[1]),
                     "MTK_VCP_VA_ENC_NODE", "mtk-vcodec-enc", MTKVCP_ENC_NODE);
        mtkvcp_log("codec nodes: dec=%s enc=%s", node_path[0], node_path[1]);
        nodes_done = 1;
    }
    pthread_mutex_unlock(&node_lock);
}

/* Node the decode contexts and the exported unbound surfaces must share. */
const char *mtkvcp_dec_node(void)
{
    resolve_nodes();
    return node_path[0];
}

const char *mtkvcp_enc_node(void)
{
    resolve_nodes();
    return node_path[1];
}

/* Open a V4L2 node and verify it is a V4L2 M2M device. */
int mtkvcp_v4l2_open(const char *node)
{
    struct v4l2_capability cap;
    int fd = open(node, O_RDWR | O_NONBLOCK);
    int r;
    if (fd < 0)
        return -errno;
    r = mtkvcp_xioctl(fd, VIDIOC_QUERYCAP, &cap);
    if (r < 0) {
        close(fd);
        return r;
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE)) {
        close(fd);
        return -ENODEV;
    }
    return fd;
}

int mtkvcp_v4l2_s_fmt_planes(int fd, enum v4l2_buf_type type,
                             uint32_t fourcc, int w, int h, int planes,
                             size_t *sizeimage_out)
{
    struct v4l2_format f;
    memset(&f, 0, sizeof(f));
    f.type = type;
    f.fmt.pix_mp.width = (uint32_t)w;
    f.fmt.pix_mp.height = (uint32_t)h;
    f.fmt.pix_mp.pixelformat = fourcc;
    f.fmt.pix_mp.field = V4L2_FIELD_NONE;
    f.fmt.pix_mp.num_planes = (uint32_t)(planes > 0 ? planes : 1);
    /* Leave sizeimage 0: the driver negotiates a sane default
     * (and the encoder CAPTURE path floors zero proposals). */
    int r = mtkvcp_xioctl(fd, VIDIOC_S_FMT, &f);
    if (r < 0)
        return r;
    /* The driver may legitimately pick the other family (10-bit streams
     * always come back as MT2T/P010), so compare the layout, not the fourcc. */
    if (planes <= 1 && f.fmt.pix_mp.pixelformat != fourcc)
        return -EINVAL;
    if (sizeimage_out)
        *sizeimage_out = f.fmt.pix_mp.plane_fmt[0].sizeimage;
    return 0;
}

int mtkvcp_v4l2_s_fmt(int fd, enum v4l2_buf_type type,
                             uint32_t fourcc, int w, int h,
                             size_t *sizeimage_out)
{
    return mtkvcp_v4l2_s_fmt_planes(fd, type, fourcc, w, h, 1, sizeimage_out);
}

/* REQBUFS for MMAP; returns granted count or negative errno. */
int mtkvcp_v4l2_reqbufs(int fd, enum v4l2_buf_type type, int count)
{
    struct v4l2_requestbuffers rb;
    memset(&rb, 0, sizeof(rb));
    rb.count = (uint32_t)count;
    rb.type = type;
    rb.memory = V4L2_MEMORY_MMAP;
    int r = mtkvcp_xioctl(fd, VIDIOC_REQBUFS, &rb);
    if (r < 0)
        return r;
    return (int)rb.count;
}

/* Same as above with an explicit memory type (MMAP or DMABUF). */
int mtkvcp_v4l2_reqbufs_mem(int fd, enum v4l2_buf_type type, int count,
                            uint32_t memory)
{
    struct v4l2_requestbuffers rb;
    memset(&rb, 0, sizeof(rb));
    rb.count = (uint32_t)count;
    rb.type = type;
    rb.memory = memory;
    int r = mtkvcp_xioctl(fd, VIDIOC_REQBUFS, &rb);
    if (r < 0)
        return r;
    return (int)rb.count;
}

int mtkvcp_v4l2_stream(int fd, enum v4l2_buf_type type, int on)
{
    enum v4l2_buf_type t = type;
    return mtkvcp_xioctl(fd, on ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &t);
}

int mtkvcp_v4l2_subscribe(int fd, uint32_t evtype)
{
    struct v4l2_event_subscription sub;
    memset(&sub, 0, sizeof(sub));
    sub.type = evtype;
    return mtkvcp_xioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
}

/* Set one integer control. Vendor controls are not in the standard set, so
 * they go through the extended-control form. */
int mtkvcp_v4l2_s_ctrl(int fd, uint32_t id, int32_t value)
{
    struct v4l2_ext_control ec;
    struct v4l2_ext_controls ecs;

    memset(&ec, 0, sizeof(ec));
    memset(&ecs, 0, sizeof(ecs));
    ec.id = id;
    ec.value = value;
    ecs.ctrl_class = V4L2_CTRL_ID2WHICH(id);
    ecs.count = 1;
    ecs.controls = &ec;
    return mtkvcp_xioctl(fd, VIDIOC_S_EXT_CTRLS, &ecs);
}
