#include "network/ws_protocol.h"

#include <stdio.h>
#include <string.h>

static int failures;

static void expect_true(int cond, const char *name, int line)
{
    if (!cond) {
        fprintf(stderr, "FAIL:%d: %s\n", line, name);
        failures++;
    }
}

#define EXPECT_TRUE(cond) expect_true((cond), #cond, __LINE__)
#define EXPECT_EQ(a, b) expect_true((a) == (b), #a " == " #b, __LINE__)

static size_t build_client_frame(uint8_t opcode,
                                 const uint8_t *payload,
                                 size_t payload_len,
                                 int masked,
                                 int fin,
                                 uint8_t rsv_bits,
                                 uint8_t *out,
                                 size_t out_cap)
{
    const uint8_t mask[4] = {0x12U, 0x34U, 0x56U, 0x78U};
    size_t pos = 0U;
    size_t header_len = payload_len < 126U ? 2U : 4U;
    size_t mask_len = masked ? sizeof(mask) : 0U;

    if (out_cap < header_len + mask_len + payload_len)
        return 0U;

    out[pos++] = (uint8_t)((fin ? 0x80U : 0U) | rsv_bits | opcode);
    if (payload_len < 126U) {
        out[pos++] = (uint8_t)((masked ? 0x80U : 0U) | payload_len);
    } else {
        out[pos++] = (uint8_t)((masked ? 0x80U : 0U) | 126U);
        out[pos++] = (uint8_t)(payload_len >> 8U);
        out[pos++] = (uint8_t)(payload_len & 0xFFU);
    }

    if (masked) {
        memcpy(&out[pos], mask, sizeof(mask));
        pos += sizeof(mask);
        for (size_t i = 0; i < payload_len; i++)
            out[pos + i] = payload[i] ^ mask[i % 4U];
    } else {
        memcpy(&out[pos], payload, payload_len);
    }

    return pos + payload_len;
}

static void test_masked_short_text(void)
{
    const uint8_t payload[] = "{\"ping\":123}";
    uint8_t frame_buf[128];
    cam_ws_frame_t frame;
    size_t frame_len = build_client_frame(CAM_WS_OPCODE_TEXT,
                                          payload,
                                          strlen((const char *)payload),
                                          1,
                                          1,
                                          0U,
                                          frame_buf,
                                          sizeof(frame_buf));

    EXPECT_EQ(cam_ws_parse_client_frame(frame_buf, frame_len, &frame),
              CAM_WS_PARSE_OK);
    EXPECT_EQ(frame.opcode, CAM_WS_OPCODE_TEXT);
    EXPECT_EQ(frame.payload_len, strlen((const char *)payload));
    EXPECT_TRUE(strcmp((const char *)frame.payload, (const char *)payload) == 0);
    EXPECT_EQ(frame.frame_len, frame_len);
}

static void test_extended_126_text(void)
{
    uint8_t payload[130];
    uint8_t frame_buf[160];
    cam_ws_frame_t frame;

    memset(payload, 'a', sizeof(payload));
    size_t frame_len = build_client_frame(CAM_WS_OPCODE_TEXT,
                                          payload,
                                          sizeof(payload),
                                          1,
                                          1,
                                          0U,
                                          frame_buf,
                                          sizeof(frame_buf));

    EXPECT_EQ(cam_ws_parse_client_frame(frame_buf, frame_len, &frame),
              CAM_WS_PARSE_OK);
    EXPECT_EQ(frame.payload_len, sizeof(payload));
    EXPECT_TRUE(memcmp(frame.payload, payload, sizeof(payload)) == 0);
    EXPECT_EQ(frame.payload[sizeof(payload)], '\0');
}

static void test_control_frames(void)
{
    const uint8_t ping_payload[] = "ab";
    const uint8_t close_payload[] = {0x03U, 0xE8U};
    uint8_t frame_buf[32];
    cam_ws_frame_t frame;
    size_t frame_len;

    frame_len = build_client_frame(CAM_WS_OPCODE_PING,
                                   ping_payload,
                                   strlen((const char *)ping_payload),
                                   1,
                                   1,
                                   0U,
                                   frame_buf,
                                   sizeof(frame_buf));
    EXPECT_EQ(cam_ws_parse_client_frame(frame_buf, frame_len, &frame),
              CAM_WS_PARSE_OK);
    EXPECT_EQ(frame.opcode, CAM_WS_OPCODE_PING);
    EXPECT_TRUE(memcmp(frame.payload, ping_payload, frame.payload_len) == 0);

    frame_len = build_client_frame(CAM_WS_OPCODE_CLOSE,
                                   close_payload,
                                   sizeof(close_payload),
                                   1,
                                   1,
                                   0U,
                                   frame_buf,
                                   sizeof(frame_buf));
    EXPECT_EQ(cam_ws_parse_client_frame(frame_buf, frame_len, &frame),
              CAM_WS_PARSE_OK);
    EXPECT_EQ(frame.opcode, CAM_WS_OPCODE_CLOSE);
    EXPECT_EQ(frame.payload_len, sizeof(close_payload));
}

static void test_incomplete_frame(void)
{
    const uint8_t payload[] = "abc";
    uint8_t frame_buf[32];
    cam_ws_frame_t frame;
    size_t frame_len = build_client_frame(CAM_WS_OPCODE_TEXT,
                                          payload,
                                          strlen((const char *)payload),
                                          1,
                                          1,
                                          0U,
                                          frame_buf,
                                          sizeof(frame_buf));

    EXPECT_EQ(cam_ws_parse_client_frame(frame_buf, frame_len - 1U, &frame),
              CAM_WS_PARSE_INCOMPLETE);
}

static void test_rejected_client_frames(void)
{
    const uint8_t payload[] = "abc";
    uint8_t frame_buf[1100];
    uint8_t large_payload[CAM_WS_MAX_PAYLOAD + 1U];
    cam_ws_frame_t frame;
    size_t frame_len;

    frame_len = build_client_frame(CAM_WS_OPCODE_TEXT,
                                   payload,
                                   strlen((const char *)payload),
                                   0,
                                   1,
                                   0U,
                                   frame_buf,
                                   sizeof(frame_buf));
    EXPECT_EQ(cam_ws_parse_client_frame(frame_buf, frame_len, &frame),
              CAM_WS_PARSE_PROTOCOL_ERROR);

    frame_len = build_client_frame(CAM_WS_OPCODE_TEXT,
                                   payload,
                                   strlen((const char *)payload),
                                   1,
                                   0,
                                   0U,
                                   frame_buf,
                                   sizeof(frame_buf));
    EXPECT_EQ(cam_ws_parse_client_frame(frame_buf, frame_len, &frame),
              CAM_WS_PARSE_PROTOCOL_ERROR);

    frame_len = build_client_frame(0x02U,
                                   payload,
                                   strlen((const char *)payload),
                                   1,
                                   1,
                                   0U,
                                   frame_buf,
                                   sizeof(frame_buf));
    EXPECT_EQ(cam_ws_parse_client_frame(frame_buf, frame_len, &frame),
              CAM_WS_PARSE_PROTOCOL_ERROR);

    memset(large_payload, 'x', sizeof(large_payload));
    frame_len = build_client_frame(CAM_WS_OPCODE_TEXT,
                                   large_payload,
                                   sizeof(large_payload),
                                   1,
                                   1,
                                   0U,
                                   frame_buf,
                                   sizeof(frame_buf));
    EXPECT_EQ(cam_ws_parse_client_frame(frame_buf, frame_len, &frame),
              CAM_WS_PARSE_PAYLOAD_TOO_LARGE);
}

static void test_server_headers(void)
{
    uint8_t header[10];
    size_t header_len = 0U;

    EXPECT_EQ(cam_ws_build_server_header(CAM_WS_OPCODE_TEXT,
                                         5U,
                                         header,
                                         &header_len), 0);
    EXPECT_EQ(header_len, 2U);
    EXPECT_EQ(header[0], 0x81U);
    EXPECT_EQ(header[1], 5U);

    EXPECT_EQ(cam_ws_build_server_header(CAM_WS_OPCODE_TEXT,
                                         126U,
                                         header,
                                         &header_len), 0);
    EXPECT_EQ(header_len, 4U);
    EXPECT_EQ(header[0], 0x81U);
    EXPECT_EQ(header[1], 126U);
    EXPECT_EQ(header[2], 0U);
    EXPECT_EQ(header[3], 126U);

    EXPECT_EQ(cam_ws_build_server_header(CAM_WS_OPCODE_PING,
                                         126U,
                                         header,
                                         &header_len), -1);
}

int main(void)
{
    test_masked_short_text();
    test_extended_126_text();
    test_control_frames();
    test_incomplete_frame();
    test_rejected_client_frames();
    test_server_headers();

    if (failures != 0) {
        fprintf(stderr, "ws_protocol_test: %d failure(s)\n", failures);
        return 1;
    }

    printf("ws_protocol_test: OK (%zu-byte max client payload)\n",
           (size_t)CAM_WS_MAX_PAYLOAD);
    return 0;
}
