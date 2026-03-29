/* yolo_detect.cpp — YOLOv6-Lite-S NCNN 推理线程
 *
 * Core 0 专用，从帧缓冲取帧 → 降采样 → 推理 → NMS → 写入 g_state
 *
 * 模型: yolov6lite_s.ncnn (256x192 输入, BGR, out0 输出)
 * 输出格式: out0 每行 [cx, cy, w, h, obj_conf, cls0..cls79]
 */

#include "yolo_detect.h"

#include "ncnn/net.h"
#include "ncnn/mat.h"

extern "C" {
#include "app_state.h"
#include "config.h"
#include "tracker.h"
}

extern tracker_t g_tracker;

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <sys/time.h>

/* ── 检测框结构（内部使用） ── */
struct BBox {
    float x1, y1, x2, y2;  /* 像素坐标，相对于 YOLO 输入尺寸 */
    float score;
    int   label;
};

/* ── 工具函数 ── */
static double get_time_ms()
{
    static double t0 = 0;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double now = tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
    if (t0 == 0) t0 = now;
    return now - t0;
}

static float compute_iou(const BBox &a, const BBox &b)
{
    float x1 = std::max(a.x1, b.x1);
    float y1 = std::max(a.y1, b.y1);
    float x2 = std::min(a.x2, b.x2);
    float y2 = std::min(a.y2, b.y2);
    float inter = std::max(0.f, x2 - x1) * std::max(0.f, y2 - y1);
    float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
    float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
    return inter / (area_a + area_b - inter + 1e-6f);
}

static void nms_sorted(std::vector<BBox> &bboxes, float threshold, int max_det)
{
    std::sort(bboxes.begin(), bboxes.end(),
        [](const BBox &a, const BBox &b) { return a.score > b.score; });

    std::vector<bool> removed(bboxes.size(), false);
    for (size_t i = 0; i < bboxes.size(); i++) {
        if (removed[i]) continue;
        for (size_t j = i + 1; j < bboxes.size(); j++) {
            if (removed[j]) continue;
            if (compute_iou(bboxes[i], bboxes[j]) > threshold)
                removed[j] = true;
        }
    }

    std::vector<BBox> result;
    for (size_t i = 0; i < bboxes.size(); i++)
        if (!removed[i]) result.push_back(bboxes[i]);
    bboxes = result;
    if ((int)bboxes.size() > max_det)
        bboxes.resize(max_det);
}

/* ── COCO 80 类名 ── */
static const char *coco_names[80] = {
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier",
    "toothbrush"
};

/* ── 解码模型输出 ── */
static void decode_output(const ncnn::Mat &out, float conf_thresh,
                          const int *enabled_classes, int enabled_count,
                          std::vector<BBox> &proposals)
{
    for (int i = 0; i < out.h; i++) {
        const float *row = out.row(i);
        float obj_conf = row[4];

        /* 找最大类别分数 */
        float max_cls_score = -1.f;
        int max_cls_id = -1;
        for (int c = 0; c < 80; c++) {
            float s = row[5 + c];
            if (s > max_cls_score) {
                max_cls_score = s;
                max_cls_id = c;
            }
        }

        float score = obj_conf * max_cls_score;
        if (score <= conf_thresh) continue;

        /* 类别过滤 */
        if (enabled_count > 0 && enabled_count < 80) {
            bool found = false;
            for (int k = 0; k < enabled_count; k++) {
                if (enabled_classes[k] == max_cls_id) { found = true; break; }
            }
            if (!found) continue;
        }

        float cx = row[0];
        float cy = row[1];
        float w  = row[2];
        float h  = row[3];

        BBox box;
        box.x1 = cx - w / 2.f;
        box.y1 = cy - h / 2.f;
        box.x2 = cx + w / 2.f;
        box.y2 = cy + h / 2.f;
        box.score = score;
        box.label = max_cls_id;
        proposals.push_back(box);
    }
}

/* ── 全局 NCNN 网络 ── */
static ncnn::Net *g_net = NULL;

/* ── 公开接口 ── */

extern "C" {

int yolo_init(void)
{
    printf("[yolo] Loading model: %s\n", MODEL_PARAM_PATH);

    g_net = new ncnn::Net();
    g_net->opt.use_vulkan_compute = false;
    g_net->opt.num_threads = 1;  /* Core 0 单线程 */
    g_net->opt.use_packing_layout = true;

    if (g_net->load_param(MODEL_PARAM_PATH) != 0) {
        fprintf(stderr, "[yolo] Failed to load param\n");
        delete g_net; g_net = NULL;
        return -1;
    }
    if (g_net->load_model(MODEL_BIN_PATH) != 0) {
        fprintf(stderr, "[yolo] Failed to load bin\n");
        delete g_net; g_net = NULL;
        return -1;
    }

    /* 预热 3 次 */
    printf("[yolo] Warming up...\n");
    ncnn::Mat dummy(YOLO_INPUT_W, YOLO_INPUT_H, 3);
    dummy.fill(0.f);
    for (int w = 0; w < 3; w++) {
        ncnn::Extractor ex = g_net->create_extractor();
        ex.input("in0", dummy);
        ncnn::Mat out;
        ex.extract("out0", out);
    }

    printf("[yolo] Model loaded and warmed up\n");
    return 0;
}

void *yolo_thread(void *arg)
{
    (void)arg;
    printf("[yolo] Thread started on Core 0\n");

    if (!g_net) {
        fprintf(stderr, "[yolo] Model not loaded, thread exiting\n");
        return NULL;
    }

    const float mean_vals[3] = {0.f, 0.f, 0.f};
    const float norm_vals[3] = {1/255.f, 1/255.f, 1/255.f};

    unsigned int last_seq = 0;

    while (!g_state.quit) {

        if (!g_state.yolo_enabled) {
            usleep(100000);
            continue;
        }

        int frame_idx = g_state.latest_frame_idx;
        unsigned int cur_seq = g_state.frame_seq;
        if (frame_idx < 0 || cur_seq == last_seq) {
            usleep(5000);
            continue;
        }
        last_seq = cur_seq;

        /* 取帧时记录 offset 快照 */
        int snap_xoff = g_state.frame_offsets[frame_idx].xoff;
        int snap_yoff = g_state.frame_offsets[frame_idx].yoff;

        uint8_t *rgb_data = g_frame_pool[frame_idx];
        if (!rgb_data) continue;

        double t0 = get_time_ms();

        ncnn::Mat in = ncnn::Mat::from_pixels_resize(
            rgb_data, ncnn::Mat::PIXEL_RGB2BGR,
            IMG_W, IMG_H, YOLO_INPUT_W, YOLO_INPUT_H);
        in.substract_mean_normalize(mean_vals, norm_vals);

        ncnn::Extractor ex = g_net->create_extractor();
        ex.input("in0", in);
        ncnn::Mat out;
        ex.extract("out0", out);

        double t_infer = get_time_ms();

        int enabled_classes[YOLO_NUM_CLASSES];
        int enabled_count = 0;
        int max_det = YOLO_MAX_DET;
        float conf_thresh = YOLO_CONF_THRESH;

        pthread_mutex_lock(&g_state.settings_mutex);
        enabled_count = g_state.enabled_class_count;
        if (enabled_count > 0 && enabled_count < YOLO_NUM_CLASSES)
            memcpy(enabled_classes, g_state.enabled_classes,
                   enabled_count * sizeof(int));
        max_det = g_state.max_det;
        pthread_mutex_unlock(&g_state.settings_mutex);

        std::vector<BBox> proposals;
        decode_output(out, conf_thresh, enabled_classes, enabled_count, proposals);
        nms_sorted(proposals, YOLO_NMS_THRESH, max_det);

        double t1 = get_time_ms();

        pthread_mutex_lock(&g_state.det_mutex);
        g_state.det_count = 0;
        for (size_t i = 0; i < proposals.size() && i < (size_t)YOLO_MAX_DET; i++) {
            const BBox &b = proposals[i];
            detection_t *d = &g_state.detections[g_state.det_count];

            float scale_x = (float)IMG_W / YOLO_INPUT_W;
            float scale_y = (float)IMG_H / YOLO_INPUT_H;
            float x1 = std::max(0.f, std::min(b.x1 * scale_x, (float)IMG_W));
            float y1 = std::max(0.f, std::min(b.y1 * scale_y, (float)IMG_H));
            float x2 = std::max(0.f, std::min(b.x2 * scale_x, (float)IMG_W));
            float y2 = std::max(0.f, std::min(b.y2 * scale_y, (float)IMG_H));

            d->bbox[0] = x1 / IMG_W;
            d->bbox[1] = y1 / IMG_H;
            d->bbox[2] = (x2 - x1) / IMG_W;
            d->bbox[3] = (y2 - y1) / IMG_H;
            d->class_id = b.label;
            d->confidence = b.score;
            d->track_id = (int)i;
            d->is_target = 0;
            g_state.det_count++;
        }

        /* 用快照 offset 调用 tracker 观测更新 */
        {
            float time_sec = (float)(t1 / 1000.0);
            tracker_update_detections(&g_tracker,
                                     g_state.detections,
                                     g_state.det_count,
                                     snap_xoff, snap_yoff,
                                     time_sec);
            g_state.tracking_active = tracker_is_active(&g_tracker);
            g_state.tracking_target_id = tracker_get_target_id(&g_tracker);
        }

        g_state.det_seq = cur_seq;
        pthread_mutex_unlock(&g_state.det_mutex);

        printf("[yolo] infer=%.1fms total=%.1fms det=%d\n",
               t_infer - t0, t1 - t0, (int)proposals.size());
    }

    printf("[yolo] Thread exiting\n");
    return NULL;
}

void yolo_close(void)
{
    if (g_net) {
        delete g_net;
        g_net = NULL;
    }
}

} /* extern "C" */
