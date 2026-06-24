#include "cam_system/app_state.h"
#include "cam_system/tracker.h"
#include "network/cmd_handler.h"

#include <pthread.h>
#include <stdio.h>
#include <string.h>

app_state_t g_state;
tracker_t g_tracker;
uint8_t *g_frame_pool[FRAME_BUF_CNT];
uint8_t *g_cached_pool[FRAME_BUF_CNT];
volatile sig_atomic_t g_app_signal_exit_requested;

void app_state_init(app_state_t *s)
{
    memset(s, 0, sizeof(*s));
    s->latest_frame_idx = -1;
    s->tracking_target_id = -1;
    s->track_target_id = -1;
    s->enabled_class_count = 1;
    s->enabled_classes[0] = 0;
    s->max_det = YOLO_MAX_DET;
    pthread_mutex_init(&s->frame_mutex, NULL);
    pthread_mutex_init(&s->pred_mutex, NULL);
    pthread_mutex_init(&s->jpeg_mutex, NULL);
    pthread_mutex_init(&s->det_mutex, NULL);
    pthread_mutex_init(&s->cmd_mutex, NULL);
    pthread_mutex_init(&s->settings_mutex, NULL);
}

static void app_state_destroy(app_state_t *s)
{
    pthread_mutex_destroy(&s->frame_mutex);
    pthread_mutex_destroy(&s->pred_mutex);
    pthread_mutex_destroy(&s->jpeg_mutex);
    pthread_mutex_destroy(&s->det_mutex);
    pthread_mutex_destroy(&s->cmd_mutex);
    pthread_mutex_destroy(&s->settings_mutex);
}

static int expect_contains(const char *name, const char *haystack,
                           const char *needle)
{
    if (strstr(haystack, needle))
        return 0;

    fprintf(stderr, "FAIL:%s: missing '%s'\n%s\n", name, needle, haystack);
    return 1;
}

static int expect_int(const char *name, int actual, int expected)
{
    if (actual == expected)
        return 0;

    fprintf(stderr, "FAIL:%s: expected %d, got %d\n",
            name, expected, actual);
    return 1;
}

int main(void)
{
    char response[512];
    int failures = 0;

    app_state_init(&g_state);

    memset(response, 0, sizeof(response));
    cmd_handle_api("POST /api/camera_bad HTTP/1.1\r\n",
                   "{\"enabled\":true}", response, sizeof(response));
    failures += expect_contains("camera_bad", response, "404 Not Found");
    failures += expect_int("camera_bad_state",
                           app_state_load_int(&g_state.cam_enabled), 0);

    memset(response, 0, sizeof(response));
    cmd_handle_api("GET /api/camera HTTP/1.1\r\n",
                   "{\"enabled\":true}", response, sizeof(response));
    failures += expect_contains("camera_get", response, "404 Not Found");
    failures += expect_int("camera_get_state",
                           app_state_load_int(&g_state.cam_enabled), 0);

    memset(response, 0, sizeof(response));
    cmd_handle_api("POST /api/camera?client=test HTTP/1.1\r\n",
                   "{\"enabled\":true}", response, sizeof(response));
    failures += expect_contains("camera_query", response, "200 OK");
    failures += expect_int("camera_query_state",
                           app_state_load_int(&g_state.cam_enabled), 1);

    memset(response, 0, sizeof(response));
    cmd_handle_api("POST /api/settings_extra HTTP/1.1\r\n",
                   "{\"max_det\":1}", response, sizeof(response));
    failures += expect_contains("settings_extra", response, "404 Not Found");

    app_state_destroy(&g_state);

    if (failures) {
        fprintf(stderr, "cmd_handler_route_test: %d failure(s)\n", failures);
        return 1;
    }

    printf("cmd_handler_route_test: OK\n");
    return 0;
}
