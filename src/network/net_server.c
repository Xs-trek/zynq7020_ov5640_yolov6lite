/* net_server.c — HTTP MJPEG 推流 + REST API + WebSocket
 *
 * 单线程 select() 事件循环
 * - GET /stream      → MJPEG multipart 推流
 * - POST /api/xxx    → JSON 命令处理
 * - GET /ws          → WebSocket 升级 → 检测结果推送
 *
 * 依赖: cJSON (third_party/)
 */
#include "network/net_server.h"
#include "network/cmd_handler.h"
#include "network/ws_protocol.h"
#include "cam_system/app_state.h"
#include "cam_system/config.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <linux/sockios.h>
#include <netinet/tcp.h>
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
    uint32_t mjpeg_drop_count; /* transport-backpressure dropped frames */
} client_t;

static int listen_fd = -1;
static client_t clients[MAX_HTTP_CLIENTS + MAX_WS_CLIENTS];
static int client_count = 0;
static int server_running = 0;

static int send_all_limited(int fd, const void *buf, size_t len,
                            int wait_budget_ms);

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
    return send_all_limited(fd, buf, len, HTTP_SEND_WAIT_BUDGET_MS);
}

static int send_all_limited(int fd, const void *buf, size_t len, int wait_budget_ms)
{
    const char *p = (const char *)buf;
    size_t sent = 0;
    int waited_ms = 0;

    while (sent < len) {
        ssize_t n = send(fd, p + sent, len - sent, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                fd_set wr_set;
                struct timeval tv = { .tv_sec = 0, .tv_usec = 5000 };

                if (waited_ms >= wait_budget_ms)
                    return -1;

                FD_ZERO(&wr_set);
                FD_SET(fd, &wr_set);
                if (select(fd + 1, NULL, &wr_set, NULL, &tv) < 0 &&
                    errno != EINTR)
                    return -1;
                waited_ms += 5;
                continue;
            }
            return -1;
        }
        if (n == 0) return -1;
        sent += n;
    }
    return 0;
}

static void configure_mjpeg_socket(int fd)
{
    int opt = 1;
    int sndbuf = MJPEG_SOCKET_SNDBUF_BYTES;

    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt)) < 0)
        fprintf(stderr, "[net] TCP_NODELAY failed: %s\n", strerror(errno));
    if (setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0)
        fprintf(stderr, "[net] SO_SNDBUF failed: %s\n", strerror(errno));
}

static int mjpeg_socket_backlogged(int fd)
{
    int outq = 0;

    if (ioctl(fd, SIOCOUTQ, &outq) < 0)
        return 0;

    return outq > MJPEG_OUTQ_DROP_THRESHOLD_BYTES;
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

    if (mjpeg_socket_backlogged(c->fd)) {
        unsigned int skipped = g_state.jpeg_seq - c->last_jpeg_seq;
        if (skipped == 0)
            skipped = 1;
        c->mjpeg_drop_count += skipped;
        c->last_jpeg_seq = g_state.jpeg_seq;
        pthread_mutex_unlock(&g_state.jpeg_mutex);
        return 0;
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
        "Content-Length: %zu\r\n"
        "X-Frame-Seq: %u\r\n"
        "X-Dropped-Frames: %u\r\n\r\n",
        MJPEG_BOUNDARY, sz, c->last_jpeg_seq, c->mjpeg_drop_count);
    if (hlen < 0 || hlen >= (int)sizeof(hdr))
        return -1;

    int ret = 0;
    if (send_all_limited(c->fd, hdr, hlen, MJPEG_SEND_WAIT_BUDGET_MS) < 0 ||
        send_all_limited(c->fd, c->mjpeg_buf, sz, MJPEG_SEND_WAIT_BUDGET_MS) < 0) {
        ret = -1;
    }
    return ret;
}

/* ── WebSocket 帧发送 ── */
static int ws_send_frame(int fd, uint8_t opcode, const void *payload, size_t len)
{
    uint8_t hdr[10];
    size_t hdr_len;

    if (cam_ws_build_server_header(opcode, len, hdr, &hdr_len) < 0)
        return -1;
    if (send_all_limited(fd, hdr, hdr_len, 50) < 0)
        return -1;
    if (len > 0 && send_all_limited(fd, payload, len, 50) < 0)
        return -1;
    return 0;
}

static int ws_send_text(int fd, const char *text, size_t len)
{
    return ws_send_frame(fd, CAM_WS_OPCODE_TEXT, text, len);
}

static int ws_send_pong(int fd, const uint8_t *payload, size_t len)
{
    return ws_send_frame(fd, CAM_WS_OPCODE_PONG, payload, len);
}

static int ws_send_close_status(int fd, uint16_t code)
{
    uint8_t payload[2] = {
        (uint8_t)(code >> 8),
        (uint8_t)(code & 0xFF)
    };

    return ws_send_frame(fd, CAM_WS_OPCODE_CLOSE, payload, sizeof(payload));
}

static int ws_handle_text_message(client_t *c, const cam_ws_frame_t *frame)
{
    const char *msg = (const char *)frame->payload;
    const char *ping = strstr(msg, "\"ping\"");
    const char *colon;
    char *endp;
    double ping_ts;
    char pong[128];
    int plen;

    if (!ping)
        return 0;

    colon = strchr(ping, ':');
    if (!colon)
        return 0;

    errno = 0;
    ping_ts = strtod(colon + 1, &endp);
    if (errno || endp == colon + 1)
        return 0;

    plen = snprintf(pong, sizeof(pong), "{\"pong\":%.0f}", ping_ts);
    if (plen < 0 || plen >= (int)sizeof(pong))
        return -1;
    return ws_send_text(c->fd, pong, (size_t)plen);
}

static int ws_process_recv_buffer(client_t *c)
{
    size_t offset = 0U;
    size_t len = (size_t)c->recv_len;

    while (offset < len) {
        cam_ws_frame_t frame;
        cam_ws_parse_result_t ret =
            cam_ws_parse_client_frame((const uint8_t *)c->recv_buf + offset,
                                      len - offset,
                                      &frame);

        if (ret == CAM_WS_PARSE_INCOMPLETE)
            break;
        if (ret == CAM_WS_PARSE_PAYLOAD_TOO_LARGE) {
            ws_send_close_status(c->fd, 1009U);
            return -1;
        }
        if (ret != CAM_WS_PARSE_OK) {
            ws_send_close_status(c->fd, 1002U);
            return -1;
        }

        if (frame.opcode == CAM_WS_OPCODE_TEXT) {
            if (ws_handle_text_message(c, &frame) < 0)
                return -1;
        } else if (frame.opcode == CAM_WS_OPCODE_PING) {
            if (ws_send_pong(c->fd, frame.payload, frame.payload_len) < 0)
                return -1;
        } else if (frame.opcode == CAM_WS_OPCODE_CLOSE) {
            ws_send_close_status(c->fd, 1000U);
            return -1;
        }

        offset += frame.frame_len;
    }

    if (offset > 0U) {
        size_t remaining = len - offset;

        if (remaining > 0U)
            memmove(c->recv_buf, c->recv_buf + offset, remaining);
        c->recv_len = (int)remaining;
        c->recv_buf[c->recv_len] = '\0';
    }

    if ((size_t)c->recv_len >= sizeof(c->recv_buf) - 1U)
        return -1;
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

    if (rlen < 0 || rlen >= (int)sizeof(response))
        return -1;
    if (send_all(c->fd, response, rlen) < 0) return -1;
    c->ws_handshake_done = 1;
    c->type = CLIENT_WS;
    return 0;
}

static int parse_content_length(const char *req, const char *headers_end)
{
    const char *p = req;

    while (p < headers_end) {
        const char *line_end = strstr(p, "\r\n");
        if (!line_end || line_end > headers_end)
            line_end = headers_end;

        if ((line_end - p) >= 15 &&
            strncasecmp(p, "Content-Length:", 15) == 0) {
            char *endp;
            long len;

            p += 15;
            while (p < line_end && (*p == ' ' || *p == '\t'))
                p++;

            errno = 0;
            len = strtol(p, &endp, 10);
            if (errno || endp == p || len < 0 ||
                len > (long)(sizeof(clients[0].recv_buf) - 1))
                return -1;
            return (int)len;
        }

        if (line_end == headers_end)
            break;
        p = line_end + 2;
    }

    return 0;
}

static int http_request_complete(const client_t *c)
{
    const char *headers_end = strstr(c->recv_buf, "\r\n\r\n");
    int header_len;
    int body_len;

    if (!headers_end)
        return 0;

    headers_end += 4;
    if (strncmp(c->recv_buf, "POST ", 5) != 0)
        return 1;

    body_len = parse_content_length(c->recv_buf, headers_end - 4);
    if (body_len < 0)
        return -1;

    header_len = (int)(headers_end - c->recv_buf);
    return c->recv_len >= header_len + body_len;
}

static void format_target_id(char *buf, size_t sz)
{
    int target_id = app_state_load_int(&g_state.tracking_target_id);

    if (target_id >= 0)
        snprintf(buf, sz, "%d", target_id);
    else
        snprintf(buf, sz, "null");
}

static int json_append(char *json, size_t sz, int pos, const char *fmt, ...)
{
    va_list ap;
    int n;

    if (pos < 0 || (size_t)pos >= sz)
        return -1;

    va_start(ap, fmt);
    n = vsnprintf(json + pos, sz - (size_t)pos, fmt, ap);
    va_end(ap);

    if (n < 0 || (size_t)n >= sz - (size_t)pos)
        return -1;
    return pos + n;
}

static int append_status_fields(char *json, size_t sz, int pos)
{
    char target_id[16];
    unsigned int status_seq = app_state_load_u32(&g_state.status_seq);

    format_target_id(target_id, sizeof(target_id));
    return json_append(json, sz, pos,
        "\"cam_enabled\":%s,"
        "\"yolo_enabled\":%s,"
        "\"tracking\":%s,"
        "\"target_id\":%s,"
        "\"fps_capture\":0,"
        "\"fps_inference\":0,"
        "\"fps_stream\":0,"
        "\"uptime_sec\":0,"
        "\"status_seq\":%u",
        app_state_load_int(&g_state.cam_enabled) ? "true" : "false",
        app_state_load_int(&g_state.yolo_enabled) ? "true" : "false",
        app_state_load_int(&g_state.tracking_active) ? "true" : "false",
        target_id,
        status_seq);
}

static int make_status_body(char *body, size_t sz)
{
    int pos = 0;

    pos = json_append(body, sz, pos, "{\"status\":{");
    pos = append_status_fields(body, sz, pos);
    pos = json_append(body, sz, pos, "},");
    pos = append_status_fields(body, sz, pos);
    pos = json_append(body, sz, pos, "}");
    return pos;
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
            "Cache-Control: no-store, no-cache, must-revalidate, max-age=0\r\n"
            "Pragma: no-cache\r\n"
            "Connection: keep-alive\r\n"
            "Access-Control-Allow-Origin: *\r\n\r\n";
        c->mjpeg_buf_cap = FRAME_SIZE;
        c->mjpeg_buf = (uint8_t *)malloc(c->mjpeg_buf_cap);
        if (!c->mjpeg_buf) {
            const char *oom_resp =
                "HTTP/1.1 503 Service Unavailable\r\n"
                "Content-Length: 0\r\n"
                "Connection: close\r\n\r\n";

            fprintf(stderr, "[net] mjpeg_buf malloc failed\n");
            send_all(c->fd, oom_resp, strlen(oom_resp));
            c->type = CLIENT_API;
            return;
        }

        configure_mjpeg_socket(c->fd);
        if (send_all(c->fd, resp, strlen(resp)) == 0) {
            c->type = CLIENT_MJPEG;
            c->last_jpeg_seq = 0;
            c->mjpeg_drop_count = 0;
        } else {
            c->type = CLIENT_API;
        }

    } else if (strncmp(req, "GET /api/status", 15) == 0) {
        /* 设备状态 — APP 连接测试用 */
        char body[1024];
        int body_len = make_status_body(body, sizeof(body));

        if (body_len < 0) {
            const char *resp =
                "HTTP/1.1 500 Internal Server Error\r\n"
                "Content-Length: 0\r\n\r\n";
            send_all(c->fd, resp, strlen(resp));
            c->type = CLIENT_API;
            return;
        }

        char resp[1280];
        snprintf(resp, sizeof(resp),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Content-Length: %zu\r\n\r\n%s",
            (size_t)body_len, body);
        send_all(c->fd, resp, strlen(resp));
        c->type = CLIENT_API;

    } else if (strncmp(req, "GET /ws", 7) == 0 ||
               strstr(req, "Upgrade: websocket")) {
        /* WebSocket 升级 */
        int upgraded = 0;
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
                    upgraded = (ws_handshake(c, key) == 0);
                }
            }
        }
        if (!upgraded) {
            const char *resp =
                "HTTP/1.1 400 Bad Request\r\n"
                "Content-Length: 0\r\n\r\n";
            send_all(c->fd, resp, strlen(resp));
            c->type = CLIENT_API;
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
    app_state_store_int(&server_running, 1);
    printf("[net] HTTP server listening on port %d\n", HTTP_PORT);

    while (app_state_load_int(&server_running) &&
           !app_state_load_int(&g_state.quit)) {
        if (g_app_signal_exit_requested) {
            printf("[net] Signal requested shutdown\n");
            app_state_store_int(&g_state.quit, 1);
            app_state_store_int(&server_running, 0);
            break;
        }

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

                        if (c->type == CLIENT_NEW) {
                            int complete = http_request_complete(c);
                            if (complete < 0) {
                                remove = 1;
                            } else if (complete > 0) {
                                handle_http_request(c);
                                if (c->type == CLIENT_API) {
                                    remove = 1;
                                }
                                c->recv_len = 0;
                            }
                        }
                        /* WS: 接收 APP 发来的消息 */
                        if (c->type == CLIENT_WS && c->ws_handshake_done) {
                            if (ws_process_recv_buffer(c) < 0)
                                remove = 1;
                        }
                    }
                } else {
                    remove = 1;
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

        /* 广播检测结果或状态变化给 WS 客户端 */
        static unsigned int last_det_seq = 0;
        static unsigned int last_status_seq = 0;
        unsigned int det_seq = app_state_load_u32(&g_state.det_seq);
        unsigned int status_seq = app_state_load_u32(&g_state.status_seq);

        if (det_seq != last_det_seq || status_seq != last_status_seq) {
            last_det_seq = det_seq;
            last_status_seq = status_seq;
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

    pos = json_append(json, sizeof(json), pos, "{\"status\":{");
    pos = append_status_fields(json, sizeof(json), pos);
    pos = json_append(json, sizeof(json), pos, "},\"detections\":[");
    if (pos < 0)
        return;

    pthread_mutex_lock(&g_state.det_mutex);
    int count = g_state.det_count;
    for (int i = 0; i < count && i < YOLO_MAX_DET; i++) {
        detection_t *d = &g_state.detections[i];
        const char *class_name = coco_names_lookup(d->class_id);
        if (i > 0)
            pos = json_append(json, sizeof(json), pos, ",");
        pos = json_append(json, sizeof(json), pos,
            "{\"bbox\":[%.1f,%.1f,%.1f,%.1f],"
            "\"class_name\":\"%s\","
            "\"confidence\":%.3f,"
            "\"id\":%d,"
            "\"is_target\":%s}",
            d->bbox[0] * 640.0f, d->bbox[1] * 480.0f,
            d->bbox[2] * 640.0f, d->bbox[3] * 480.0f,
            class_name, d->confidence, d->track_id,
            d->is_target ? "true" : "false");
        if (pos < 0) {
            pthread_mutex_unlock(&g_state.det_mutex);
            return;
        }
    }
    pthread_mutex_unlock(&g_state.det_mutex);

    pos = json_append(json, sizeof(json), pos, "]");
    if (pos < 0)
        return;

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

        pos = json_append(json, sizeof(json), pos,
            ",\"predicted_target\":{"
            "\"bbox\":[%.1f,%.1f,%.1f,%.1f],"
            "\"class_name\":\"%s\","
            "\"id\":%d,"
            "\"screen_cx\":%.3f,"
            "\"screen_cy\":%.3f}",
            px, py, ppw, pph,
            pname, pid, pcx, pcy);
        if (pos < 0)
            return;
    }

    pos = json_append(json, sizeof(json), pos, "}");
    if (pos < 0)
        return;

    for (int i = 0; i < client_count; ) {
        if (clients[i].type == CLIENT_WS && clients[i].ws_handshake_done &&
            ws_send_text(clients[i].fd, json, (size_t)pos) < 0) {
            client_remove(i);
            continue;
        }
        i++;
    }
}

void net_server_stop(void)
{
    app_state_store_int(&server_running, 0);
}
