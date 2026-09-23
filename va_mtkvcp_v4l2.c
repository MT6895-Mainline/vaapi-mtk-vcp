/* SPDX-License-Identifier: MIT */
/* Small V4L2 M2M helpers. Every ioctl retries on EINTR; errors are
 * returned as negative errno. No logging here (caller decides). */
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/videodev2.h>
#include <linux/dma-heap.h>
#include <linux/dma-buf.h>

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

int mtkvcp_xioctl(int fd, unsigned long req, void *arg)
{
    int r;
    do {
        r = ioctl(fd, req, arg);
    } while (r < 0 && errno == EINTR);
    return r < 0 ? -errno : 0;
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

int mtkvcp_v4l2_s_fmt(int fd, enum v4l2_buf_type type,
                             uint32_t fourcc, int w, int h,
                             size_t *sizeimage_out)
{
    struct v4l2_format f;
    memset(&f, 0, sizeof(f));
    f.type = type;
    f.fmt.pix_mp.width = (uint32_t)w;
    f.fmt.pix_mp.height = (uint32_t)h;
    f.fmt.pix_mp.pixelformat = fourcc;
    f.fmt.pix_mp.field = V4L2_FIELD_NONE;
    f.fmt.pix_mp.num_planes = 1;
    /* Leave sizeimage 0: the driver negotiates a sane default
     * (and the encoder CAPTURE path floors zero proposals). */
    int r = mtkvcp_xioctl(fd, VIDIOC_S_FMT, &f);
    if (r < 0)
        return r;
    if (f.fmt.pix_mp.pixelformat != fourcc)
        return -EINVAL;
    if (sizeimage_out)
        *sizeimage_out = f.fmt.pix_mp.plane_fmt[0].sizeimage;
    return 0;
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
