/* V4L2 capture backend adapter.
 *
 * Product path: Xilinx VIPP/frmbuf through /dev/videoX mmap buffers.
 * Removed direct-register fallback code is available only through git history.
 */
#include "platform/hal_frmbuf.h"
#include "cam_system/config.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/media-bus-format.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

struct v4l2_slot {
    void *addr;
    size_t len;
    int queued;
};

static int video_fd = -1;
static struct v4l2_slot v4l2_slots[FRAME_BUF_CNT];
static int v4l2_streaming = 0;
static enum v4l2_buf_type v4l2_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
static int v4l2_mplane = 0;

static int xioctl(int fd, unsigned long request, void *arg)
{
    int ret;

    do {
        ret = ioctl(fd, request, arg);
    } while (ret < 0 && errno == EINTR);

    return ret;
}

static int cap_subdev_name_matches(const char *entry, const char *needle)
{
    char name_path[512];
    char name[128];
    int fd;
    ssize_t n;

    snprintf(name_path, sizeof(name_path), "/sys/class/video4linux/%s/name",
             entry);
    fd = open(name_path, O_RDONLY);
    if (fd < 0)
        return 0;

    n = read(fd, name, sizeof(name) - 1);
    close(fd);
    if (n <= 0)
        return 0;

    name[n] = '\0';
    return strstr(name, needle) != NULL;
}

static int cap_set_subdev_format(const char *needle, unsigned int pad,
                                 uint32_t code)
{
    DIR *dir;
    struct dirent *de;
    char path[512];
    int fd;
    int ret = -1;

    dir = opendir("/sys/class/video4linux");
    if (!dir)
        return -1;

    while ((de = readdir(dir)) != NULL) {
        struct v4l2_subdev_format fmt;

        if (strncmp(de->d_name, "v4l-subdev", 10) != 0)
            continue;
        if (!cap_subdev_name_matches(de->d_name, needle))
            continue;

        snprintf(path, sizeof(path), "/dev/%s", de->d_name);
        fd = open(path, O_RDWR);
        if (fd < 0) {
            fprintf(stderr, "[capture-backend] open %s failed: %s\n",
                    path, strerror(errno));
            break;
        }

        memset(&fmt, 0, sizeof(fmt));
        fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
        fmt.pad = pad;
        fmt.format.width = IMG_W;
        fmt.format.height = IMG_H;
        fmt.format.code = code;
        fmt.format.field = V4L2_FIELD_NONE;
        fmt.format.colorspace = V4L2_COLORSPACE_SRGB;

        if (xioctl(fd, VIDIOC_SUBDEV_S_FMT, &fmt) < 0) {
            fprintf(stderr,
                    "[capture-backend] VIDIOC_SUBDEV_S_FMT %s pad=%u code=0x%04x failed: %s\n",
                    path, pad, code, strerror(errno));
            close(fd);
            break;
        }

        printf("[capture-backend] media format %s pad=%u code=0x%04x %ux%u\n",
               path, pad, fmt.format.code, fmt.format.width, fmt.format.height);
        close(fd);
        ret = 0;
        break;
    }

    closedir(dir);
    if (ret < 0)
        fprintf(stderr, "[capture-backend] subdev '%s' pad=%u not configured\n",
                needle, pad);
    return ret;
}

static int cap_configure_media_graph(void)
{
    const char *skip = getenv("CAM_CAPTURE_SKIP_MEDIA_SETUP");

    if (skip && strcmp(skip, "1") == 0)
        return 0;

    if (cap_set_subdev_format("cam-ov5640-control", 0,
                              MEDIA_BUS_FMT_RGB565_2X8_LE) < 0)
        return -1;
    if (cap_set_subdev_format("ov5640_to_lcd", 0,
                              MEDIA_BUS_FMT_RGB565_2X8_LE) < 0)
        return -1;
    if (cap_set_subdev_format("ov5640_to_lcd", 1,
                              MEDIA_BUS_FMT_RGB888_1X24) < 0)
        return -1;
    if (cap_set_subdev_format("axis-subset-converter", 0,
                              MEDIA_BUS_FMT_RGB888_1X24) < 0)
        return -1;
    if (cap_set_subdev_format("axis-subset-converter", 1,
                              MEDIA_BUS_FMT_RBG888_1X24) < 0)
        return -1;

    return 0;
}

static int cap_select_v4l2_type(int fd, const char *path)
{
    struct v4l2_capability cap;
    uint32_t caps;

    memset(&cap, 0, sizeof(cap));
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        fprintf(stderr, "[capture-backend] VIDIOC_QUERYCAP %s failed: %s\n",
                path, strerror(errno));
        return -1;
    }

    caps = cap.device_caps ? cap.device_caps : cap.capabilities;
    if (!(caps & V4L2_CAP_STREAMING)) {
        fprintf(stderr, "[capture-backend] %s does not support streaming\n",
                path);
        return -1;
    }

    if (caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
        v4l2_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        v4l2_mplane = 1;
    } else if (caps & V4L2_CAP_VIDEO_CAPTURE) {
        v4l2_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        v4l2_mplane = 0;
    } else {
        fprintf(stderr, "[capture-backend] %s is not a capture device caps=0x%08x\n",
                path, caps);
        return -1;
    }

    printf("[capture-backend] using V4L2 device %s (%s, %s)\n",
           path, (const char *)cap.card,
           v4l2_mplane ? "mplane" : "single-plane");
    return 0;
}

static int cap_open_video_device(void)
{
    const char *forced = getenv("CAM_CAPTURE_DEV");
    char path[32];

    if (forced && forced[0] != '\0') {
        video_fd = open(forced, O_RDWR | O_NONBLOCK);
        if (video_fd >= 0 && cap_select_v4l2_type(video_fd, forced) == 0)
            return 0;
        fprintf(stderr, "[capture-backend] open %s failed: %s\n",
                forced, video_fd < 0 ? strerror(errno) : "unsupported device");
        if (video_fd >= 0) {
            close(video_fd);
            video_fd = -1;
        }
        return -1;
    }

    for (int i = 0; i < 10; i++) {
        snprintf(path, sizeof(path), "/dev/video%d", i);
        video_fd = open(path, O_RDWR | O_NONBLOCK);
        if (video_fd < 0)
            continue;

        if (cap_select_v4l2_type(video_fd, path) == 0)
            return 0;

        close(video_fd);
        video_fd = -1;
    }

    fprintf(stderr, "[capture-backend] no V4L2 capture device found\n");
    return -1;
}

int frmbuf_init(void)
{
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;

    if (video_fd >= 0)
        return 0;
    if (cap_open_video_device() < 0)
        return -1;
    if (cap_configure_media_graph() < 0)
        return -1;

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = v4l2_type;
    if (v4l2_mplane) {
        fmt.fmt.pix_mp.width = IMG_W;
        fmt.fmt.pix_mp.height = IMG_H;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_RGB24;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = 1;
        fmt.fmt.pix_mp.plane_fmt[0].bytesperline = IMG_W * IMG_BPP;
        fmt.fmt.pix_mp.plane_fmt[0].sizeimage = FRAME_SIZE;
    } else {
        fmt.fmt.pix.width = IMG_W;
        fmt.fmt.pix.height = IMG_H;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        fmt.fmt.pix.bytesperline = IMG_W * IMG_BPP;
        fmt.fmt.pix.sizeimage = FRAME_SIZE;
    }

    if (xioctl(video_fd, VIDIOC_S_FMT, &fmt) < 0) {
        fprintf(stderr, "[capture-backend] VIDIOC_S_FMT RGB24 %dx%d failed: %s\n",
                IMG_W, IMG_H, strerror(errno));
        return -1;
    }

    if (v4l2_mplane) {
        if (fmt.fmt.pix_mp.width != IMG_W || fmt.fmt.pix_mp.height != IMG_H ||
            fmt.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_RGB24 ||
            fmt.fmt.pix_mp.num_planes < 1 ||
            fmt.fmt.pix_mp.plane_fmt[0].bytesperline < IMG_W * IMG_BPP ||
            fmt.fmt.pix_mp.plane_fmt[0].sizeimage < FRAME_SIZE) {
            fprintf(stderr,
                    "[capture-backend] unexpected mplane format %ux%u fourcc=0x%08x planes=%u stride=%u size=%u\n",
                    fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
                    fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.num_planes,
                    fmt.fmt.pix_mp.plane_fmt[0].bytesperline,
                    fmt.fmt.pix_mp.plane_fmt[0].sizeimage);
            return -1;
        }
    } else {
        if (fmt.fmt.pix.width != IMG_W || fmt.fmt.pix.height != IMG_H ||
            fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB24 ||
            fmt.fmt.pix.bytesperline < IMG_W * IMG_BPP ||
            fmt.fmt.pix.sizeimage < FRAME_SIZE) {
            fprintf(stderr,
                    "[capture-backend] unexpected format %ux%u fourcc=0x%08x stride=%u size=%u\n",
                    fmt.fmt.pix.width, fmt.fmt.pix.height,
                    fmt.fmt.pix.pixelformat, fmt.fmt.pix.bytesperline,
                    fmt.fmt.pix.sizeimage);
            return -1;
        }
    }

    memset(&req, 0, sizeof(req));
    req.count = FRAME_BUF_CNT;
    req.type = v4l2_type;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(video_fd, VIDIOC_REQBUFS, &req) < 0) {
        fprintf(stderr, "[capture-backend] VIDIOC_REQBUFS failed: %s\n",
                strerror(errno));
        return -1;
    }
    if (req.count < FRAME_BUF_CNT) {
        fprintf(stderr, "[capture-backend] insufficient V4L2 buffers: %u\n",
                req.count);
        return -1;
    }

    for (int i = 0; i < FRAME_BUF_CNT; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        unsigned int length;
        unsigned int offset;

        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));
        buf.type = v4l2_type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (v4l2_mplane) {
            buf.length = 1;
            buf.m.planes = planes;
        }

        if (xioctl(video_fd, VIDIOC_QUERYBUF, &buf) < 0) {
            fprintf(stderr, "[capture-backend] VIDIOC_QUERYBUF %d failed: %s\n",
                    i, strerror(errno));
            return -1;
        }

        length = v4l2_mplane ? planes[0].length : buf.length;
        offset = v4l2_mplane ? planes[0].m.mem_offset : buf.m.offset;
        v4l2_slots[i].len = length;
        v4l2_slots[i].addr = mmap(NULL, length, PROT_READ | PROT_WRITE,
                                  MAP_SHARED, video_fd, offset);
        if (v4l2_slots[i].addr == MAP_FAILED) {
            fprintf(stderr, "[capture-backend] mmap buffer %d failed: %s\n",
                    i, strerror(errno));
            v4l2_slots[i].addr = NULL;
            return -1;
        }

        if (xioctl(video_fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "[capture-backend] VIDIOC_QBUF %d failed: %s\n",
                    i, strerror(errno));
            return -1;
        }
        v4l2_slots[i].queued = 1;
    }

    if (xioctl(video_fd, VIDIOC_STREAMON, &v4l2_type) < 0) {
        fprintf(stderr, "[capture-backend] VIDIOC_STREAMON failed: %s\n",
                strerror(errno));
        return -1;
    }

    v4l2_streaming = 1;
    printf("[capture-backend] V4L2 capture ready: %dx%d RGB24 buffers=%d\n",
           IMG_W, IMG_H, FRAME_BUF_CNT);
    return 0;
}

uint8_t *frmbuf_get_buffer(int buf_idx)
{
    if (buf_idx < 0 || buf_idx >= FRAME_BUF_CNT)
        return NULL;
    return (uint8_t *)v4l2_slots[buf_idx].addr;
}

static int cap_queue_v4l2_index(int buf_idx)
{
    struct v4l2_buffer buf;
    struct v4l2_plane planes[VIDEO_MAX_PLANES];

    if (buf_idx < 0 || buf_idx >= FRAME_BUF_CNT || v4l2_slots[buf_idx].queued)
        return -1;

    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));
    buf.type = v4l2_type;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = (unsigned int)buf_idx;
    if (v4l2_mplane) {
        buf.length = 1;
        buf.m.planes = planes;
    }

    if (xioctl(video_fd, VIDIOC_QBUF, &buf) < 0) {
        fprintf(stderr, "[capture-backend] VIDIOC_QBUF release %d failed: %s\n",
                buf_idx, strerror(errno));
        return -1;
    }

    v4l2_slots[buf_idx].queued = 1;
    return 0;
}

static void cap_queue_v4l2_index_best_effort(int buf_idx)
{
    if (buf_idx < 0 || buf_idx >= FRAME_BUF_CNT)
        return;
    if (v4l2_slots[buf_idx].queued)
        return;

    if (cap_queue_v4l2_index(buf_idx) < 0) {
        fprintf(stderr,
                "[capture-backend] best-effort QBUF cleanup %d failed\n",
                buf_idx);
    }
}

int frmbuf_capture_frame(int preferred_idx, int timeout_ms)
{
    struct pollfd pfd;
    int latest_idx = -1;
    int ret;

    (void)preferred_idx;

    if (video_fd < 0) {
        fprintf(stderr, "[capture-backend] not initialized\n");
        return -1;
    }

    pfd.fd = video_fd;
    pfd.events = POLLIN;
    pfd.revents = 0;
    ret = poll(&pfd, 1, timeout_ms > 0 ? timeout_ms : 2000);
    if (ret <= 0) {
        fprintf(stderr, "[capture-backend] V4L2 frame %s\n",
                ret == 0 ? "timeout" : strerror(errno));
        return -1;
    }

    for (int drained = 0; drained < FRAME_BUF_CNT; drained++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[VIDEO_MAX_PLANES];
        unsigned int bytesused;

        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));
        buf.type = v4l2_type;
        buf.memory = V4L2_MEMORY_MMAP;
        if (v4l2_mplane) {
            buf.length = 1;
            buf.m.planes = planes;
        }

        if (xioctl(video_fd, VIDIOC_DQBUF, &buf) < 0) {
            if ((errno == EAGAIN || errno == EWOULDBLOCK) && latest_idx >= 0)
                break;
            fprintf(stderr, "[capture-backend] VIDIOC_DQBUF failed: %s\n",
                    strerror(errno));
            cap_queue_v4l2_index_best_effort(latest_idx);
            return -1;
        }

        bytesused = v4l2_mplane ? planes[0].bytesused : buf.bytesused;
        if (buf.index >= FRAME_BUF_CNT) {
            fprintf(stderr, "[capture-backend] invalid buffer index=%u bytes=%u\n",
                    buf.index, bytesused);
            cap_queue_v4l2_index_best_effort(latest_idx);
            return -1;
        }

        v4l2_slots[buf.index].queued = 0;
        if (bytesused < FRAME_SIZE) {
            fprintf(stderr, "[capture-backend] invalid buffer index=%u bytes=%u\n",
                    buf.index, bytesused);
            cap_queue_v4l2_index_best_effort((int)buf.index);
            cap_queue_v4l2_index_best_effort(latest_idx);
            return -1;
        }

        if (latest_idx >= 0 && cap_queue_v4l2_index(latest_idx) < 0) {
            cap_queue_v4l2_index_best_effort((int)buf.index);
            cap_queue_v4l2_index_best_effort(latest_idx);
            return -1;
        }
        latest_idx = (int)buf.index;
    }

    return latest_idx;
}

int frmbuf_release_frame(int buf_idx)
{
    return cap_queue_v4l2_index(buf_idx);
}

void frmbuf_stop(void)
{
    if (video_fd >= 0 && v4l2_streaming) {
        if (xioctl(video_fd, VIDIOC_STREAMOFF, &v4l2_type) < 0)
            fprintf(stderr, "[capture-backend] VIDIOC_STREAMOFF failed: %s\n",
                    strerror(errno));
        v4l2_streaming = 0;
    }
}

void frmbuf_close(void)
{
    frmbuf_stop();

    for (int i = 0; i < FRAME_BUF_CNT; i++) {
        if (v4l2_slots[i].addr) {
            munmap(v4l2_slots[i].addr, v4l2_slots[i].len);
            v4l2_slots[i].addr = NULL;
            v4l2_slots[i].len = 0;
            v4l2_slots[i].queued = 0;
        }
    }
    if (video_fd >= 0) {
        close(video_fd);
        video_fd = -1;
    }
}
