#include "optical_flow.h"

extern "C" {
#include "config.h"
}

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <vector>
#include <algorithm>
#include <stdio.h>
#include <math.h>

static cv::Mat prev_gray;
static std::vector<cv::Point2f> prev_pts;
static float last_scr_cx = 0.0f;
static float last_scr_cy = 0.0f;
static int of_initialized = 0;

/* 记录光流在历史帧上的绝对位置，用于与延迟到达的 YOLO 结果做同帧比较 */
typedef struct {
    unsigned int seq;
    float abs_x;
    float abs_y;
} of_hist_t;

static of_hist_t g_hist[64];
static int g_hist_pos = 0;

static float clampf_local(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

extern "C" void of_init(void)
{
    prev_gray.release();
    prev_pts.clear();
    last_scr_cx = 0.0f;
    last_scr_cy = 0.0f;
    of_initialized = 0;
}

extern "C" int of_is_tracking(void)
{
    return (of_initialized && prev_pts.size() >= 5) ? 1 : 0;
}

extern "C" void of_reset(const uint8_t* rgb_data,
                         float scr_cx, float scr_cy,
                         float scr_w, float scr_h)
{
    cv::Mat rgb(IMG_H, IMG_W, CV_8UC3, (void*)rgb_data);
    cv::cvtColor(rgb, prev_gray, cv::COLOR_RGB2GRAY);

    int px = (int)((scr_cx - scr_w * 0.5f) * IMG_W);
    int py = (int)((scr_cy - scr_h * 0.5f) * IMG_H);
    int pw = (int)(scr_w * IMG_W);
    int ph = (int)(scr_h * IMG_H);

    px = std::max(0, std::min(px, IMG_W - 1));
    py = std::max(0, std::min(py, IMG_H - 1));
    pw = std::max(1, std::min(pw, IMG_W - px));
    ph = std::max(1, std::min(ph, IMG_H - py));

    cv::Mat mask = cv::Mat::zeros(IMG_H, IMG_W, CV_8UC1);
    cv::Rect roi(px, py, pw, ph);
    mask(roi).setTo(255);

    prev_pts.clear();
    cv::goodFeaturesToTrack(prev_gray, prev_pts,
                            30,     /* maxCorners */
                            0.01,   /* qualityLevel */
                            5.0,    /* minDistance */
                            mask);

    last_scr_cx = clampf_local(scr_cx, 0.0f, 1.0f);
    last_scr_cy = clampf_local(scr_cy, 0.0f, 1.0f);
    of_initialized = (prev_pts.size() >= 5) ? 1 : 0;

    printf("[OF] Reset tracking with %d points\n", (int)prev_pts.size());
}

extern "C" int of_track(const uint8_t* rgb_data,
                        unsigned int frame_seq,
                        int cur_snap_xoff, int cur_snap_yoff,
                        float* out_abs_x, float* out_abs_y)
{
    if (!of_initialized || prev_pts.size() < 5 || !out_abs_x || !out_abs_y)
        return 0;

    cv::Mat rgb(IMG_H, IMG_W, CV_8UC3, (void*)rgb_data);
    cv::Mat curr_gray;
    cv::cvtColor(rgb, curr_gray, cv::COLOR_RGB2GRAY);

    std::vector<cv::Point2f> curr_pts;
    std::vector<uchar> status;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(prev_gray, curr_gray,
                             prev_pts, curr_pts,
                             status, err);

    std::vector<float> dxs;
    std::vector<float> dys;
    std::vector<cv::Point2f> good_pts;

    for (size_t i = 0; i < status.size(); i++) {
        if (status[i]) {
            dxs.push_back(curr_pts[i].x - prev_pts[i].x);
            dys.push_back(curr_pts[i].y - prev_pts[i].y);
            good_pts.push_back(curr_pts[i]);
        }
    }

    if (good_pts.size() < 5) {
        of_initialized = 0;
        prev_pts.clear();
        return 0;
    }

    std::sort(dxs.begin(), dxs.end());
    std::sort(dys.begin(), dys.end());

    float med_dx = dxs[dxs.size() / 2];
    float med_dy = dys[dys.size() / 2];

    last_scr_cx += med_dx / (float)IMG_W;
    last_scr_cy += med_dy / (float)IMG_H;

    last_scr_cx = clampf_local(last_scr_cx, 0.0f, 1.0f);
    last_scr_cy = clampf_local(last_scr_cy, 0.0f, 1.0f);

    *out_abs_x = (1.0f - last_scr_cx) * (float)IMG_W + (float)cur_snap_xoff;
    *out_abs_y = (1.0f - last_scr_cy) * (float)IMG_H + (float)cur_snap_yoff;

    /* 写入历史轨迹，供 YOLO 延迟校准使用 */
    g_hist[g_hist_pos].seq   = frame_seq;
    g_hist[g_hist_pos].abs_x = *out_abs_x;
    g_hist[g_hist_pos].abs_y = *out_abs_y;
    g_hist_pos = (g_hist_pos + 1) % 64;

    prev_gray = curr_gray.clone();
    prev_pts = good_pts;

    return 1;
}

extern "C" void of_calibrate(unsigned int yolo_seq,
                             float yolo_abs_x, float yolo_abs_y,
                             float w, float h,
                             const uint8_t* curr_rgb,
                             int cur_snap_x, int cur_snap_y)
{
    /* 如果光流当前未初始化，直接用 YOLO 的绝对位置投影到当前帧并重建 */
    if (!of_initialized) {
        float curr_scr_cx = 1.0f - (yolo_abs_x - (float)cur_snap_x) / (float)IMG_W;
        float curr_scr_cy = 1.0f - (yolo_abs_y - (float)cur_snap_y) / (float)IMG_H;

        /* 为了容忍 YOLO 延迟带来的目标运动，初始化时适当放大 ROI */
        float rw = clampf_local(w * 1.6f, 0.05f, 1.0f);
        float rh = clampf_local(h * 1.6f, 0.05f, 1.0f);

        of_reset(curr_rgb, curr_scr_cx, curr_scr_cy, rw, rh);
        return;
    }

    /* 找到与该 YOLO 历史帧对应的光流估计 */
    int found = 0;
    float hist_abs_x = 0.0f, hist_abs_y = 0.0f;
    for (int i = 0; i < 64; i++) {
        if (g_hist[i].seq == yolo_seq) {
            hist_abs_x = g_hist[i].abs_x;
            hist_abs_y = g_hist[i].abs_y;
            found = 1;
            break;
        }
    }

    if (!found) {
        return;
    }

    /* 计算同一历史时刻的漂移误差 */
    float drift_x = yolo_abs_x - hist_abs_x;
    float drift_y = yolo_abs_y - hist_abs_y;

    /* 软校正：将漂移映射回当前光流内部 screen 状态 */
    const float cal_gain = 0.25f;
    last_scr_cx -= cal_gain * drift_x / (float)IMG_W;
    last_scr_cy -= cal_gain * drift_y / (float)IMG_H;

    last_scr_cx = clampf_local(last_scr_cx, 0.0f, 1.0f);
    last_scr_cy = clampf_local(last_scr_cy, 0.0f, 1.0f);

    /* 若漂移明显过大，认为光流可能跟偏，使用当前帧重新建点
     * 阈值与目标框尺寸相关，避免小误差也频繁触发 reset */
    float reset_thresh_x = std::max(50.0f, 0.6f * w * (float)IMG_W);
    float reset_thresh_y = std::max(50.0f, 0.6f * h * (float)IMG_H);

    if (fabsf(drift_x) > reset_thresh_x || fabsf(drift_y) > reset_thresh_y) {
        float rw = clampf_local(w * 1.3f, 0.05f, 1.0f);
        float rh = clampf_local(h * 1.3f, 0.05f, 1.0f);

        of_reset(curr_rgb, last_scr_cx, last_scr_cy, rw, rh);
    }
}
