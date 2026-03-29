/* cmd_handler.c — HTTP API 命令处理*/
#include "cmd_handler.h"
#include "hal_ov5640.h"
#include "app_state.h"
#include "tracker.h"
#include "config.h"
#include "cJSON.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "test.h"

extern tracker_t g_tracker;

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
    cJSON *json = cJSON_Parse(body);

    if (strstr(request, "POST /api/camera")) {
        cJSON *en = json ? cJSON_GetObjectItem(json, "enabled") : NULL;
        if (en && cJSON_IsBool(en)) {
            int enable = cJSON_IsTrue(en);
            g_state.cam_enabled = enable;
            printf("[cmd] Camera %s\n", enable ? "ON" : "OFF");
            make_response(response, resp_size, 200, "OK",
                         "{\"status\":\"ok\"}");
        } else {
            make_response(response, resp_size, 400, "Bad Request",
                         "{\"error\":\"missing enabled\"}");
        }

    } else if (strstr(request, "POST /api/yolo")) {
        cJSON *en = json ? cJSON_GetObjectItem(json, "enabled") : NULL;
        if (en && cJSON_IsBool(en)) {
            g_state.yolo_enabled = cJSON_IsTrue(en);
            printf("[cmd] YOLO %s\n",
                   g_state.yolo_enabled ? "ON" : "OFF");
            make_response(response, resp_size, 200, "OK",
                         "{\"status\":\"ok\"}");
        } else {
            make_response(response, resp_size, 400, "Bad Request",
                         "{\"error\":\"missing enabled\"}");
        }

    } else if (strstr(request, "POST /api/track")) {
        cJSON *act = json ? cJSON_GetObjectItem(json, "action") : NULL;
        if (act && cJSON_IsString(act)) {

            if (strcmp(act->valuestring, "test") == 0) {
                /* ISP 路径测试：先停 tracker，再启动 test */
                tracker_stop(&g_tracker);
                test_start(TEST_MODE_ISP_PATH);
                printf("[cmd] ISP path test started\n");
                make_response(response, resp_size, 200, "OK",
                             "{\"status\":\"ok\",\"message\":\"ISP path test started\"}");
            } else if (strcmp(act->valuestring, "stop") == 0) {
                /* stop 同时停 test 和 tracker */
                if (test_is_active()) {
                    test_stop();
                    printf("[cmd] Test stopped\n");
                } else {
                    pthread_mutex_lock(&g_state.cmd_mutex);
                    g_state.track_cmd = TRACK_CMD_STOP;
                    pthread_mutex_unlock(&g_state.cmd_mutex);
                    tracker_handle_cmd(&g_tracker);
                    printf("[cmd] Tracker stopped\n");
                }
                make_response(response, resp_size, 200, "OK",
                             "{\"status\":\"ok\"}");
            } else {
                /* select / start 走原有逻辑 */
                if (test_is_active()) {
                    test_stop();  /* 停掉测试再操作 tracker */
                }
                pthread_mutex_lock(&g_state.cmd_mutex);
                if (strcmp(act->valuestring, "start") == 0) {
                    g_state.track_cmd = TRACK_CMD_START;
                } else if (strcmp(act->valuestring, "select") == 0) {
                    g_state.track_cmd = TRACK_CMD_SELECT;
                    cJSON *tid = cJSON_GetObjectItem(json, "target_id");
                    g_state.track_target_id = tid ? tid->valueint : -1;
                }
                pthread_mutex_unlock(&g_state.cmd_mutex);

                tracker_handle_cmd(&g_tracker);

                printf("[cmd] Track: %s (active=%d target=%d)\n",
                       act->valuestring,
                       tracker_is_active(&g_tracker),
                       tracker_get_target_id(&g_tracker));
                make_response(response, resp_size, 200, "OK",
                             "{\"status\":\"ok\"}");
            }
        } else {
            make_response(response, resp_size, 400, "Bad Request",
                         "{\"error\":\"missing action\"}");
        }

    } else if (strstr(request, "POST /api/zoom")) {
        make_response(response, resp_size, 200, "OK",
                     "{\"status\":\"ok\"}");

    } else if (strstr(request, "POST /api/tune")) {
        if (!json) {
            make_response(response, resp_size, 400, "Bad Request",
                         "{\"error\":\"invalid json\"}");
        } else {
            const char *keys[] = {"omega_stable","omega_maneuver","omega_coast",
                                  "kf_q_pos","kf_q_vel","kf_r","chi2"};
            for (int k = 0; k < 7; k++) {
                cJSON *v = cJSON_GetObjectItem(json, keys[k]);
                if (v && cJSON_IsNumber(v))
                    tracker_tune(&g_tracker, keys[k], (float)v->valuedouble);
            }
            make_response(response, resp_size, 200, "OK",
                         "{\"status\":\"ok\"}");
        }

    } else if (strstr(request, "POST /api/settings")) {
        pthread_mutex_lock(&g_state.settings_mutex);
        cJSON *cls = json ? cJSON_GetObjectItem(json, "classes") : NULL;
        if (cls && cJSON_IsArray(cls)) {
            g_state.enabled_class_count = 0;
            cJSON *item;
            cJSON_ArrayForEach(item, cls) {
                if (g_state.enabled_class_count < YOLO_NUM_CLASSES) {
                    g_state.enabled_classes[g_state.enabled_class_count++] =
                        item->valueint;
                }
            }
        }
        cJSON *md = json ? cJSON_GetObjectItem(json, "max_det") : NULL;
        if (md && cJSON_IsNumber(md)) {
            g_state.max_det = md->valueint;
        }
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
