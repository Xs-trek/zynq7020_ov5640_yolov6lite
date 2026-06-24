/* cmd_handler.c — HTTP API 命令处理*/
#include "network/cmd_handler.h"
#include "platform/hal_ov5640.h"
#include "cam_system/app_state.h"
#include "cam_system/tracker.h"
#include "cam_system/config.h"
#include "cjson/cJSON.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern tracker_t g_tracker;

typedef enum {
    API_ROUTE_UNKNOWN = 0,
    API_ROUTE_CAMERA,
    API_ROUTE_YOLO,
    API_ROUTE_TRACK,
    API_ROUTE_ZOOM,
    API_ROUTE_SETTINGS,
} api_route_t;

static int copy_token(char *dst, size_t dst_size,
                      const char *begin, const char *end)
{
    size_t len;

    if (!dst || dst_size == 0 || !begin || !end || end <= begin)
        return -1;

    len = (size_t)(end - begin);
    if (len >= dst_size)
        return -1;

    memcpy(dst, begin, len);
    dst[len] = '\0';
    return 0;
}

static int parse_request_line(const char *request,
                              char *method, size_t method_size,
                              char *path, size_t path_size)
{
    const char *method_end;
    const char *path_begin;
    const char *path_end;
    const char *query_begin;

    if (!request)
        return -1;

    method_end = strchr(request, ' ');
    if (!method_end)
        return -1;

    path_begin = method_end + 1;
    path_end = strchr(path_begin, ' ');
    if (!path_end)
        return -1;

    query_begin = memchr(path_begin, '?', (size_t)(path_end - path_begin));
    if (query_begin)
        path_end = query_begin;

    if (copy_token(method, method_size, request, method_end) < 0)
        return -1;
    if (copy_token(path, path_size, path_begin, path_end) < 0)
        return -1;

    return 0;
}

static api_route_t api_route_from_request(const char *request)
{
    char method[8];
    char path[64];

    if (parse_request_line(request, method, sizeof(method),
                           path, sizeof(path)) < 0)
        return API_ROUTE_UNKNOWN;
    if (strcmp(method, "POST") != 0)
        return API_ROUTE_UNKNOWN;

    if (strcmp(path, "/api/camera") == 0)
        return API_ROUTE_CAMERA;
    if (strcmp(path, "/api/yolo") == 0)
        return API_ROUTE_YOLO;
    if (strcmp(path, "/api/track") == 0)
        return API_ROUTE_TRACK;
    if (strcmp(path, "/api/zoom") == 0)
        return API_ROUTE_ZOOM;
    if (strcmp(path, "/api/settings") == 0)
        return API_ROUTE_SETTINGS;

    return API_ROUTE_UNKNOWN;
}

static int queue_track_cmd(track_cmd_t cmd, int target_id)
{
    int ret = 0;

    pthread_mutex_lock(&g_state.cmd_mutex);
    if (g_state.cmd_count >= CONTROL_CMD_QUEUE_LEN) {
        ret = -1;
    } else {
        int idx = g_state.cmd_tail;
        g_state.cmd_queue[idx] = cmd;
        g_state.cmd_target_queue[idx] = target_id;
        g_state.cmd_tail = (idx + 1) % CONTROL_CMD_QUEUE_LEN;
        g_state.cmd_count++;
    }
    pthread_mutex_unlock(&g_state.cmd_mutex);
    return ret;
}

static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void clear_detection_results(void)
{
    pthread_mutex_lock(&g_state.det_mutex);
    g_state.det_count = 0;
    g_state.det_frame_seq = 0;
    g_state.det_frame_xoff = 0;
    g_state.det_frame_yoff = 0;
    app_state_inc_u32(&g_state.det_seq);
    pthread_mutex_unlock(&g_state.det_mutex);
}

static void notify_status_changed(void)
{
    app_state_inc_u32(&g_state.status_seq);
}

static void queue_track_stop_best_effort(void)
{
    if (queue_track_cmd(TRACK_CMD_STOP, -1) == 0)
        return;

    pthread_mutex_lock(&g_state.cmd_mutex);
    g_state.track_cmd = TRACK_CMD_STOP;
    g_state.track_target_id = -1;
    pthread_mutex_unlock(&g_state.cmd_mutex);
}

static void clear_tracking_outputs(void)
{
    pthread_mutex_lock(&g_state.pred_mutex);
    g_state.pred_box.valid = 0;
    pthread_mutex_unlock(&g_state.pred_mutex);

    app_state_store_int(&g_state.tracking_active, 0);
    app_state_store_int(&g_state.tracking_target_id, -1);
}

static void clear_detection_and_tracking_outputs(void)
{
    clear_detection_results();
    clear_tracking_outputs();
}

static void make_response(char *resp, size_t sz, int code,
                          const char *status, const char *json_body)
{
    snprintf(resp, sz,
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: application/json\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Content-Length: %zu\r\n\r\n%s",
        code, status, strlen(json_body), json_body);
}

void cmd_handle_api(const char *request, const char *body,
                    char *response, size_t resp_size)
{
    api_route_t route = api_route_from_request(request);
    cJSON *json = cJSON_Parse(body ? body : "");

    if (route == API_ROUTE_CAMERA) {
        cJSON *en = json ? cJSON_GetObjectItem(json, "enabled") : NULL;
        if (en && cJSON_IsBool(en)) {
            int enable = cJSON_IsTrue(en);
            app_state_store_int(&g_state.cam_enabled, enable);
            if (!enable) {
                app_state_store_int(&g_state.yolo_enabled, 0);
                clear_detection_and_tracking_outputs();
                queue_track_stop_best_effort();
            }
            notify_status_changed();
            printf("[cmd] Camera %s\n", enable ? "ON" : "OFF");
            make_response(response, resp_size, 200, "OK",
                         "{\"status\":\"ok\"}");
        } else {
            make_response(response, resp_size, 400, "Bad Request",
                         "{\"error\":\"missing enabled\"}");
        }

    } else if (route == API_ROUTE_YOLO) {
        cJSON *en = json ? cJSON_GetObjectItem(json, "enabled") : NULL;
        if (en && cJSON_IsBool(en)) {
            int enable = cJSON_IsTrue(en);
            app_state_store_int(&g_state.yolo_enabled, enable);
            if (!enable) {
                clear_detection_and_tracking_outputs();
                queue_track_stop_best_effort();
            }
            notify_status_changed();
            printf("[cmd] YOLO %s\n",
                   enable ? "ON" : "OFF");
            make_response(response, resp_size, 200, "OK",
                         "{\"status\":\"ok\"}");
        } else {
            make_response(response, resp_size, 400, "Bad Request",
                         "{\"error\":\"missing enabled\"}");
        }

    } else if (route == API_ROUTE_TRACK) {
        cJSON *act = json ? cJSON_GetObjectItem(json, "action") : NULL;
        if (act && cJSON_IsString(act)) {

            if (strcmp(act->valuestring, "stop") == 0) {
                if (queue_track_cmd(TRACK_CMD_STOP, -1) == 0) {
                    printf("[cmd] Track stop queued\n");
                    make_response(response, resp_size, 200, "OK",
                                 "{\"status\":\"ok\"}");
                } else {
                    make_response(response, resp_size, 503, "Service Unavailable",
                                 "{\"error\":\"command queue full\"}");
                }
            } else if (strcmp(act->valuestring, "start") == 0) {
                if (queue_track_cmd(TRACK_CMD_START, -1) == 0) {
                    printf("[cmd] Track start queued\n");
                    make_response(response, resp_size, 200, "OK",
                                 "{\"status\":\"ok\"}");
                } else {
                    make_response(response, resp_size, 503, "Service Unavailable",
                                 "{\"error\":\"command queue full\"}");
                }
            } else if (strcmp(act->valuestring, "select") == 0) {
                cJSON *tid = cJSON_GetObjectItem(json, "target_id");
                if (tid && cJSON_IsNumber(tid) && tid->valueint >= 0) {
                    if (queue_track_cmd(TRACK_CMD_SELECT, tid->valueint) == 0) {
                        printf("[cmd] Track select queued target=%d\n",
                               tid->valueint);
                        make_response(response, resp_size, 200, "OK",
                                     "{\"status\":\"ok\"}");
                    } else {
                        make_response(response, resp_size, 503,
                                     "Service Unavailable",
                                     "{\"error\":\"command queue full\"}");
                    }
                } else {
                    make_response(response, resp_size, 400, "Bad Request",
                                 "{\"error\":\"invalid target_id\"}");
                }
            } else {
                make_response(response, resp_size, 400, "Bad Request",
                             "{\"error\":\"invalid action\"}");
            }
        } else {
            make_response(response, resp_size, 400, "Bad Request",
                         "{\"error\":\"missing action\"}");
        }

    } else if (route == API_ROUTE_ZOOM) {
        make_response(response, resp_size, 200, "OK",
                     "{\"status\":\"ok\"}");

    } else if (route == API_ROUTE_SETTINGS) {
        if (!json) {
            make_response(response, resp_size, 400, "Bad Request",
                         "{\"error\":\"invalid json\"}");
            return;
        }

        int new_classes[YOLO_NUM_CLASSES];
        int new_class_count = -1;
        int new_max_det = -1;
        int settings_error = 0;

        cJSON *cls = cJSON_GetObjectItem(json, "classes");
        if (cls && cJSON_IsArray(cls)) {
            new_class_count = 0;
            cJSON *item;
            cJSON_ArrayForEach(item, cls) {
                if (!cJSON_IsNumber(item) ||
                    item->valueint < 0 ||
                    item->valueint >= YOLO_NUM_CLASSES) {
                    settings_error = 1;
                    break;
                }
                if (new_class_count < YOLO_NUM_CLASSES)
                    new_classes[new_class_count++] = item->valueint;
            }
        } else if (cls) {
            settings_error = 1;
        }

        cJSON *md = cJSON_GetObjectItem(json, "max_det");
        if (md && cJSON_IsNumber(md)) {
            new_max_det = clamp_int(md->valueint, 0, YOLO_MAX_DET);
        } else if (md) {
            settings_error = 1;
        }

        if (settings_error) {
            make_response(response, resp_size, 400, "Bad Request",
                         "{\"error\":\"invalid settings\"}");
            if (json) cJSON_Delete(json);
            return;
        }

        pthread_mutex_lock(&g_state.settings_mutex);
        if (new_class_count >= 0) {
            g_state.enabled_class_count = new_class_count;
            memcpy(g_state.enabled_classes, new_classes,
                   (size_t)new_class_count * sizeof(int));
        }
        if (new_max_det >= 0)
            g_state.max_det = new_max_det;
        pthread_mutex_unlock(&g_state.settings_mutex);
        printf("[cmd] Settings updated\n");
        make_response(response, resp_size, 200, "OK",
                     "{\"status\":\"ok\"}");

    } else {
        make_response(response, resp_size, 404, "Not Found",
                     "{\"error\":\"unknown endpoint\"}");
    }

    if (json) cJSON_Delete(json);
}
