#include "network/ws_protocol.h"

#include <string.h>

static int cam_ws_opcode_supported(uint8_t opcode)
{
    return opcode == CAM_WS_OPCODE_TEXT ||
           opcode == CAM_WS_OPCODE_CLOSE ||
           opcode == CAM_WS_OPCODE_PING ||
           opcode == CAM_WS_OPCODE_PONG;
}

static int cam_ws_opcode_is_control(uint8_t opcode)
{
    return opcode >= 0x08U;
}

cam_ws_parse_result_t cam_ws_parse_client_frame(const uint8_t *buf,
                                                size_t len,
                                                cam_ws_frame_t *out)
{
    uint8_t first;
    uint8_t second;
    uint8_t opcode;
    uint8_t len_code;
    size_t header_len = 2U;
    size_t payload_len;
    const uint8_t *mask;
    const uint8_t *payload;

    if (!buf || !out)
        return CAM_WS_PARSE_PROTOCOL_ERROR;
    if (len < 2U)
        return CAM_WS_PARSE_INCOMPLETE;

    first = buf[0];
    second = buf[1];
    opcode = first & 0x0FU;
    len_code = second & 0x7FU;

    if ((first & 0x80U) == 0U || (first & 0x70U) != 0U)
        return CAM_WS_PARSE_PROTOCOL_ERROR;
    if (!cam_ws_opcode_supported(opcode))
        return CAM_WS_PARSE_PROTOCOL_ERROR;
    if ((second & 0x80U) == 0U)
        return CAM_WS_PARSE_PROTOCOL_ERROR;
    if (cam_ws_opcode_is_control(opcode) && len_code >= 126U)
        return CAM_WS_PARSE_PROTOCOL_ERROR;

    if (len_code < 126U) {
        payload_len = len_code;
    } else if (len_code == 126U) {
        if (len < 4U)
            return CAM_WS_PARSE_INCOMPLETE;
        payload_len = ((size_t)buf[2] << 8U) | (size_t)buf[3];
        header_len = 4U;
        if (payload_len < 126U)
            return CAM_WS_PARSE_PROTOCOL_ERROR;
    } else {
        return CAM_WS_PARSE_PAYLOAD_TOO_LARGE;
    }

    if (opcode == CAM_WS_OPCODE_CLOSE && payload_len == 1U)
        return CAM_WS_PARSE_PROTOCOL_ERROR;
    if (payload_len > CAM_WS_MAX_PAYLOAD)
        return CAM_WS_PARSE_PAYLOAD_TOO_LARGE;
    if (len < header_len + 4U + payload_len)
        return CAM_WS_PARSE_INCOMPLETE;

    mask = &buf[header_len];
    payload = &buf[header_len + 4U];

    out->opcode = opcode;
    out->payload_len = payload_len;
    out->frame_len = header_len + 4U + payload_len;
    for (size_t i = 0; i < payload_len; i++)
        out->payload[i] = payload[i] ^ mask[i % 4U];
    out->payload[payload_len] = '\0';
    return CAM_WS_PARSE_OK;
}

int cam_ws_build_server_header(uint8_t opcode,
                               size_t payload_len,
                               uint8_t header[10],
                               size_t *header_len)
{
    if (!header || !header_len)
        return -1;
    if (!cam_ws_opcode_supported(opcode))
        return -1;
    if (cam_ws_opcode_is_control(opcode) && payload_len > 125U)
        return -1;
    if (payload_len > 65535U)
        return -1;

    header[0] = (uint8_t)(0x80U | opcode);
    if (payload_len < 126U) {
        header[1] = (uint8_t)payload_len;
        *header_len = 2U;
    } else {
        header[1] = 126U;
        header[2] = (uint8_t)(payload_len >> 8U);
        header[3] = (uint8_t)(payload_len & 0xFFU);
        *header_len = 4U;
    }
    return 0;
}
