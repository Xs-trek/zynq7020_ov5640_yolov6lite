#ifndef CMD_HANDLER_H
#define CMD_HANDLER_H

#include <stddef.h>

/* 处理 HTTP API 请求
 * request: 完整 HTTP 请求头
 * body: POST body (JSON)
 * response: 输出 HTTP 响应（含头和body）
 * resp_size: response 缓冲区大小 */
void cmd_handle_api(const char *request, const char *body,
                    char *response, size_t resp_size);

#endif
