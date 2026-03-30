/* net_server.c — HTTP MJPEG 推流 + REST API + WebSocket
 *
 * 单线程 select() 事件循环
 * - GET /stream      → MJPEG multipart 推流
 * - POST /api/xxx    → JSON 命令处理
 * - GET /ws          → WebSocket 升级 → 检测结果推送
 *
 * 依赖: cJSON (third_party/)
 */
#include "net_server.h"
#include "cmd_handler.h"
#include "app_state.h"
#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* ── WebSocket 辅助 ── */
#include <openssl/sha.h>

/* ── COCO 类名 ── */
static const char *coco_names_tbl[80] = {
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

static const char *coco_names_lookup(int id) {
    if (id >= 0 && id < 80) return coco_names_tbl[id];
    return "unknown";
}

/* 简易 Base64 编码 */
static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static int base64_encode(const unsigned char *in, int len, char *out)
{
    int i, j = 0;
    for (i = 0; i < len - 2; i += 3) {
        out[j++] = b64_table[(in[i] >> 2) & 0x3F];
        out[j++] = b64_table[((in[i]&0x3)<<4) | ((in[i+1]>>4)&0xF)];
        out[j++] = b64_table[((in[i+1]&0xF)<<2) | ((in[i+2]>>6)&0x3)];
        out[j++] = b64_table[in[i+2] & 0x3F];
    }
    if (i < len) {
        out[j++] = b64_table[(in[i] >> 2) & 0x3F];
        if (i == len - 1) {
            out[j++] = b64_table[(in[i] & 0x3) << 4];
            out[j++] = '=';
        } else {
            out[j++] = b64_table[((in[i]&0x3)<<4) | ((in[i+1]>>4)&0xF)];
            out[j++] = b64_table[(in[i+1] & 0xF) << 2];
        }
        out[j++] = '=';
    }
    out[j] = '\0';
    return j;
}

/* ── 客户端类型 ── */
typedef enum {
    CLIENT_NEW = 0,
    CLIENT_MJPEG,
    CLIENT_WS,
    CLIENT_API
} client_type_t;

typedef struct {
    int fd;
    client_type_t type;
    uint32_t last_jpeg_seq;    /* MJPEG: 上次发送的帧序号 */
    char recv_buf[4096];
    int recv_len;
    int ws_handshake_done;
    uint8_t *mjpeg_buf;        /* 专属于这个客户端的推流缓冲区 */
    size_t mjpeg_buf_cap;      /* 缓冲区最大容量 */
} client_t;

static int listen_fd = -1;
static client_t clients[MAX_HTTP_CLIENTS + MAX_WS_CLIENTS];
static int client_count = 0;
static volatile int server_running = 0;

static void set_nonblock(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

static void client_remove(int idx)
{
    if (idx < 0 || idx >= client_count) return;
    /* 客户端断开时，回收专属缓冲区 */
    if (clients[idx].mjpeg_buf) {
        free(clients[idx].mjpeg_buf);
        clients[idx].mjpeg_buf = NULL;
    }
    close(clients[idx].fd);
    clients[idx] = clients[client_count - 1];
    client_count--;
}

static int send_all(int fd, const void *buf, size_t len)
{
    const char *p = (const char *)buf;
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = send(fd, p + sent, len - sent, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                usleep(5000); 
                continue;
            }           
            return -1; 
        }
        if (n == 0) return -1;
        sent += n;
    }
    return 0;
}

/* ── MJPEG 帧发送 ── */
static int send_mjpeg_frame(client_t *c)
{
    /* 该客户端未成功分配缓冲区，放弃帧发送 */
    if (!c->mjpeg_buf) return 0; 

    pthread_mutex_lock(&g_state.jpeg_mutex);
    if (!g_state.jpeg_buf || g_state.jpeg_size == 0 ||
        g_state.jpeg_seq == c->last_jpeg_seq) {
        pthread_mutex_unlock(&g_state.jpeg_mutex);
        return 0;  /* 没有新帧 */
    }

    /* 拷贝 JPEG 数据 */
    size_t sz = g_state.jpeg_size;
    if (sz > c->mjpeg_buf_cap) sz = c->mjpeg_buf_cap;
    memcpy(c->mjpeg_buf, g_state.jpeg_buf, sz);
    c->last_jpeg_seq = g_state.jpeg_seq;
    pthread_mutex_unlock(&g_state.jpeg_mutex);

    /* 组装 HTTP 头 */
    char hdr[256];
    int hlen = snprintf(hdr, sizeof(hdr),
        "\r\n--%s\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Length: %zu\r\n\r\n",
        MJPEG_BOUNDARY, sz);

    int ret = 0;
    if (send_all(c->fd, hdr, hlen) < 0 ||
        send_all(c->fd, c->mjpeg_buf, sz) < 0) {
        ret = -1;
    }
    return ret;
}

/* ── WebSocket 帧发送 ── */
static int ws_send_text(int fd, const char *text, size_t len)
{
    uint8_t hdr[10];
    int hdr_len;

    hdr[0] = 0x81;  /* FIN + text */
    if (len < 126) {
        hdr[1] = (uint8_t)len;
        hdr_len = 2;
    } else if (len < 65536) {
        hdr[1] = 126;
        hdr[2] = (uint8_t)(len >> 8);
        hdr[3] = (uint8_t)(len & 0xFF);
        hdr_len = 4;
    } else {
        return -1;  /* 不支持超大帧 */
    }

    if (send_all(fd, hdr, hdr_len) < 0) return -1;
    if (send_all(fd, text, len) < 0) return -1;
    return 0;
}

/* ── WebSocket 握手 ── */
static const char *WS_MAGIC = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

static int ws_handshake(client_t *c, const char *key)
{
    /* SHA1(key + magic) */
    char concat[256];
    snprintf(concat, sizeof(concat), "%s%s", key, WS_MAGIC);

    unsigned char sha1_hash[20];
    SHA1((unsigned char *)concat, strlen(concat), sha1_hash);

    char accept_key[64];
    base64_encode(sha1_hash, 20, accept_key);

    char response[512];
    int rlen = snprintf(response, sizeof(response),
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: %s\r\n\r\n",
        accept_key);

    if (send_all(c->fd, response, rlen) < 0) return -1;
    c->ws_handshake_done = 1;
    c->type = CLIENT_WS;
    return 0;
}

/* ── HTTP 请求解析 ── */
static void handle_http_request(client_t *c)
{
    char *req = c->recv_buf;

    if (strncmp(req, "GET /stream", 11) == 0) {
        /* MJPEG 推流 */
        const char *resp =
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace;boundary=" MJPEG_BOUNDARY "\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: keep-alive\r\n"
            "Access-Control-Allow-Origin: *\r\n\r\n";
        if (send_all(c->fd, resp, strlen(resp)) == 0) {
            c->type = CLIENT_MJPEG;
            c->last_jpeg_seq = 0;
            /* 当且仅当客户端请求视频流时，分配一次固定内存 */
            c->mjpeg_buf_cap = 640 * 480 * 3; 
            c->mjpeg_buf = (uint8_t *)malloc(c->mjpeg_buf_cap);
            if (!c->mjpeg_buf) {
                // 内存真爆了
                printf("[net] Warning: mjpeg_buf malloc failed!\n");
            }

        }

    } else if (strncmp(req, "GET /api/status", 15) == 0) {
        /* 设备状态 — APP 连接测试用 */
        char body[512];
        snprintf(body, sizeof(body),
            "{\"cam_enabled\":%s,"
            "\"yolo_enabled\":%s,"
            "\"tracking\":%s,"
            "\"target_id\":%d,"
            "\"zoom\":%.1f,"
            "\"fps_capture\":0,"
            "\"fps_inference\":0,"
            "\"fps_stream\":0,"
            "\"uptime_sec\":0}",
            g_state.cam_enabled ? "true" : "false",
            g_state.yolo_enabled ? "true" : "false",
            g_state.tracking_active ? "true" : "false",
            g_state.tracking_target_id,
            g_state.zoom_level / 100.0);

        char resp[768];
        snprintf(resp, sizeof(resp),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Content-Length: %zu\r\n\r\n%s",
            strlen(body), body);
        send_all(c->fd, resp, strlen(resp));
        c->type = CLIENT_API;

    } else if (strncmp(req, "GET /ws", 7) == 0 ||
               strstr(req, "Upgrade: websocket")) {
        /* WebSocket 升级 */
        char *key_start = strstr(req, "Sec-WebSocket-Key: ");
        if (key_start) {
            key_start += 19;
            char *key_end = strstr(key_start, "\r\n");
            if (key_end) {
                char key[128];
                int klen = key_end - key_start;
                if (klen < (int)sizeof(key)) {
                    memcpy(key, key_start, klen);
                    key[klen] = '\0';
                    ws_handshake(c, key);
                }
            }
        }

    } else if (strncmp(req, "POST /api/", 10) == 0) {
        /* REST API */
        char *body = strstr(req, "\r\n\r\n");
        if (body) {
            body += 4;
            char response[1024];
            cmd_handle_api(req, body, response, sizeof(response));
            send_all(c->fd, response, strlen(response));
        }
        /* API 请求处理完关闭连接 */
        c->type = CLIENT_API;

    } else if (strncmp(req, "OPTIONS ", 8) == 0) {
        /* CORS preflight */
        const char *resp =
            "HTTP/1.1 204 No Content\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
            "Access-Control-Allow-Headers: Content-Type\r\n"
            "Access-Control-Max-Age: 86400\r\n\r\n";
        send_all(c->fd, resp, strlen(resp));
        c->type = CLIENT_API;

    } else {
        const char *resp = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n";
        send_all(c->fd, resp, strlen(resp));
        c->type = CLIENT_API;
    }
}

/* ── 主事件循环 ── */
int net_server_run(void)
{
    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) { perror("[net] socket"); return -1; }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(HTTP_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[net] bind"); close(listen_fd); return -1;
    }
    if (listen(listen_fd, 8) < 0) {
        perror("[net] listen"); close(listen_fd); return -1;
    }

    set_nonblock(listen_fd);
    server_running = 1;
    printf("[net] HTTP server listening on port %d\n", HTTP_PORT);

    while (server_running && !g_state.quit) {

        fd_set rd_set;
        FD_ZERO(&rd_set);
        FD_SET(listen_fd, &rd_set);
        int max_fd = listen_fd;

        for (int i = 0; i < client_count; i++) {
            if (clients[i].type != CLIENT_MJPEG) {
                FD_SET(clients[i].fd, &rd_set);
            }
            if (clients[i].fd > max_fd) max_fd = clients[i].fd;
        }

        struct timeval tv = { .tv_sec = 0, .tv_usec = 33000 }; /* ~30fps */
        int nready = select(max_fd + 1, &rd_set, NULL, NULL, &tv);

        /* 新连接 */
        if (nready > 0 && FD_ISSET(listen_fd, &rd_set)) {
            struct sockaddr_in cli_addr;
            socklen_t cli_len = sizeof(cli_addr);
            int cfd = accept(listen_fd, (struct sockaddr *)&cli_addr, &cli_len);
            if (cfd >= 0) {
                if (client_count < MAX_HTTP_CLIENTS + MAX_WS_CLIENTS) {
                    set_nonblock(cfd);
                    client_t *nc = &clients[client_count++];
                    memset(nc, 0, sizeof(*nc));
                    nc->fd = cfd;
                    nc->type = CLIENT_NEW;
                } else {
                    close(cfd);
                }
            }
        }

        /* 处理客户端 */
        for (int i = 0; i < client_count; ) {
            client_t *c = &clients[i];
            int remove = 0;

            /* 读数据（新连接和 WS） */
            if (FD_ISSET(c->fd, &rd_set)) {
                int space = sizeof(c->recv_buf) - c->recv_len - 1;
                if (space > 0) {
                    ssize_t n = recv(c->fd, c->recv_buf + c->recv_len, space, 0);
                    if (n <= 0) {
                        remove = 1;
                    } else {
                        c->recv_len += n;
                        c->recv_buf[c->recv_len] = '\0';

                        if (c->type == CLIENT_NEW &&
                            strstr(c->recv_buf, "\r\n\r\n")) {
                            handle_http_request(c);
                            if (c->type == CLIENT_API) {
                                remove = 1;
                            }
                            c->recv_len = 0;
                        }
                        /* WS: 接收 APP 发来的消息 */
                        if (c->type == CLIENT_WS && c->ws_handshake_done) {
                            /* 简易 WS 帧解析：只处理短文本帧 */
                            uint8_t *d = (uint8_t *)c->recv_buf;
                            if (c->recv_len >= 6 && (d[0] & 0x0F) == 0x01) {
                                /* text frame */
                                int payload_len = d[1] & 0x7F;
                                if (payload_len < 126 && c->recv_len >= 6 + payload_len) {
                                    uint8_t mask[4] = {d[2], d[3], d[4], d[5]};
                                    char msg[256];
                                    int mlen = payload_len < 255 ? payload_len : 255;
                                    for (int j = 0; j < mlen; j++)
                                        msg[j] = d[6 + j] ^ mask[j % 4];
                                    msg[mlen] = '\0';

                                    /* ping → pong */
                                    if (strstr(msg, "\"ping\"")) {
                                        char *ts = strstr(msg, "\"ping\":");
                                        if (ts) {
                                            char pong[128];
                                            /* 提取 timestamp 值 */
                                            double ping_ts = 0;
                                            sscanf(ts, "\"ping\":%lf", &ping_ts);
                                            int plen = snprintf(pong, sizeof(pong),
                                                "{\"pong\":%.0f}", ping_ts);
                                            ws_send_text(c->fd, pong, plen);
                                        }
                                    }

                                    c->recv_len = 0;
                                }
                            } else if (c->recv_len >= 2 && (d[0] & 0x0F) == 0x08) {
                                /* close frame */
                                remove = 1;
                            }
                        }
                    }
                }
            }

            /* MJPEG 推帧 */
            if (!remove && c->type == CLIENT_MJPEG) {
                if (send_mjpeg_frame(c) < 0) {
                    remove = 1;
                }
            }

            if (remove) {
                client_remove(i);
            } else {
                i++;
            }
        }

        /* 广播检测结果给 WS 客户端 */
        static unsigned int last_det_seq = 0;
        if (g_state.det_seq != last_det_seq) {
            last_det_seq = g_state.det_seq;
            net_ws_broadcast_detections();
        }
    }

    /* 清理 */
    for (int i = 0; i < client_count; i++) {
        if (clients[i].mjpeg_buf) {
            free(clients[i].mjpeg_buf);
        }
        close(clients[i].fd);
    }
    client_count = 0;
    close(listen_fd);
    listen_fd = -1;
    return 0;
}

void net_ws_broadcast_detections(void)
{
    int has_ws = 0;
    for (int i = 0; i < client_count; i++) {
        if (clients[i].type == CLIENT_WS && clients[i].ws_handshake_done) {
            has_ws = 1;
            break;
        }
    }
    if (!has_ws) return;

    char json[4096];
    int pos = 0;

    pos += snprintf(json + pos, sizeof(json) - pos,
        "{\"status\":{"
        "\"cam_enabled\":%s,"
        "\"yolo_enabled\":%s,"
        "\"tracking\":%s,"
        "\"target_id\":%d,"
        "\"zoom\":%.1f"
        "},\"detections\":[",
        g_state.cam_enabled ? "true" : "false",
        g_state.yolo_enabled ? "true" : "false",
        g_state.tracking_active ? "true" : "false",
        g_state.tracking_target_id,
        g_state.zoom_level / 100.0);

    pthread_mutex_lock(&g_state.det_mutex);
    int count = g_state.det_count;
    for (int i = 0; i < count && i < YOLO_MAX_DET; i++) {
        detection_t *d = &g_state.detections[i];
        const char *class_name = coco_names_lookup(d->class_id);
        if (i > 0) pos += snprintf(json + pos, sizeof(json) - pos, ",");
        pos += snprintf(json + pos, sizeof(json) - pos,
            "{\"bbox\":[%.1f,%.1f,%.1f,%.1f],"
            "\"class_name\":\"%s\","
            "\"confidence\":%.3f,"
            "\"id\":%d,"
            "\"is_target\":%s}",
            d->bbox[0] * 640.0f, d->bbox[1] * 480.0f,
            d->bbox[2] * 640.0f, d->bbox[3] * 480.0f,
            class_name, d->confidence, d->track_id,
            d->is_target ? "true" : "false");
    }
    pthread_mutex_unlock(&g_state.det_mutex);

    pos += snprintf(json + pos, sizeof(json) - pos, "]");

    /* 追加预测框 */
    int   pvalid;
    float pcx, pcy, pw, ph;
    int   pid, pcls;

    pthread_mutex_lock(&g_state.pred_mutex);
    pvalid = g_state.pred_box.valid;
    if (pvalid) {
        pcx  = g_state.pred_box.cx;
        pcy  = g_state.pred_box.cy;
        pw   = g_state.pred_box.w;
        ph   = g_state.pred_box.h;
        pid  = g_state.pred_box.track_id;
        pcls = g_state.pred_box.class_id;
    }
    pthread_mutex_unlock(&g_state.pred_mutex);

    /* 使用局部变量进行后续耗时的查表、浮点计算和字符串格式化 */
    if (pvalid) {
        const char *pname = coco_names_lookup(pcls);

        /* 转为像素坐标 (x,y,w,h) 与 detections 格式一致 */
        float px = (pcx - pw / 2.0f) * 640.0f;
        float py = (pcy - ph / 2.0f) * 480.0f;
        float ppw = pw * 640.0f;
        float pph = ph * 480.0f;

        pos += snprintf(json + pos, sizeof(json) - pos,
            ",\"predicted_target\":{"
            "\"bbox\":[%.1f,%.1f,%.1f,%.1f],"
            "\"class_name\":\"%s\","
            "\"id\":%d,"
            "\"screen_cx\":%.3f,"
            "\"screen_cy\":%.3f}",
            px, py, ppw, pph,
            pname, pid, pcx, pcy);
    }

    pos += snprintf(json + pos, sizeof(json) - pos, "}");

    for (int i = 0; i < client_count; i++) {
        if (clients[i].type == CLIENT_WS && clients[i].ws_handshake_done)
            ws_send_text(clients[i].fd, json, pos);
    }
}

void net_server_stop(void)
{
    server_running = 0;
}
