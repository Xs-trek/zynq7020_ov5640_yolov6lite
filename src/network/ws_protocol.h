#ifndef CAM_SYSTEM_NETWORK_WS_PROTOCOL_H
#define CAM_SYSTEM_NETWORK_WS_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define CAM_WS_MAX_PAYLOAD 1024U

#define CAM_WS_OPCODE_TEXT  0x01U
#define CAM_WS_OPCODE_CLOSE 0x08U
#define CAM_WS_OPCODE_PING  0x09U
#define CAM_WS_OPCODE_PONG  0x0AU

typedef enum {
    CAM_WS_PARSE_OK = 0,
    CAM_WS_PARSE_INCOMPLETE,
    CAM_WS_PARSE_PROTOCOL_ERROR,
    CAM_WS_PARSE_PAYLOAD_TOO_LARGE,
} cam_ws_parse_result_t;

typedef struct {
    uint8_t opcode;
    uint8_t payload[CAM_WS_MAX_PAYLOAD + 1U];
    size_t payload_len;
    size_t frame_len;
} cam_ws_frame_t;

cam_ws_parse_result_t cam_ws_parse_client_frame(const uint8_t *buf,
                                                size_t len,
                                                cam_ws_frame_t *out);

int cam_ws_build_server_header(uint8_t opcode,
                               size_t payload_len,
                               uint8_t header[10],
                               size_t *header_len);

#endif
