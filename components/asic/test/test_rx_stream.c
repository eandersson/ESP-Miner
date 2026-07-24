#include <string.h>

#include "unity.h"

#include "asic_common.h"
#include "crc.h"

static void make_valid_frame(uint8_t *frame, size_t frame_size, uint8_t seed)
{
    memset(frame, 0, frame_size);
    frame[0] = 0xAA;
    frame[1] = 0x55;
    for (size_t i = 2; i + 1 < frame_size; i++) {
        frame[i] = seed + (uint8_t)i;
    }

    for (uint8_t crc = 0; crc < 32; crc++) {
        frame[frame_size - 1] = crc;
        if (crc5(frame + 2, frame_size - 2) == 0) {
            return;
        }
    }
    TEST_FAIL_MESSAGE("Unable to construct valid CRC5 frame");
}

TEST_CASE("ASIC RX stream accepts a complete valid frame", "[asic_rx]")
{
    asic_rx_stream_t stream;
    uint8_t expected[11];
    uint8_t actual[11] = {0};
    make_valid_frame(expected, sizeof(expected), 0x10);
    asic_rx_stream_reset(&stream);

    bool complete = false;
    for (size_t i = 0; i < sizeof(expected); i++) {
        complete = asic_rx_stream_push(&stream, expected[i], actual, sizeof(actual));
    }

    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(expected));
    TEST_ASSERT_EQUAL_UINT32(0, stream.discarded_bytes);
    TEST_ASSERT_EQUAL_UINT32(1, stream.frames_received);
    TEST_ASSERT_EQUAL_UINT32(0, stream.crc_errors);
}

TEST_CASE("ASIC RX stream retains a partial frame", "[asic_rx]")
{
    asic_rx_stream_t stream;
    uint8_t expected[11];
    uint8_t actual[11] = {0};
    make_valid_frame(expected, sizeof(expected), 0x20);
    asic_rx_stream_reset(&stream);

    for (size_t i = 0; i < 5; i++) {
        TEST_ASSERT_FALSE(asic_rx_stream_push(&stream, expected[i], actual, sizeof(actual)));
    }
    for (size_t i = 5; i < sizeof(expected) - 1; i++) {
        TEST_ASSERT_FALSE(asic_rx_stream_push(&stream, expected[i], actual, sizeof(actual)));
    }
    TEST_ASSERT_TRUE(asic_rx_stream_push(&stream, expected[sizeof(expected) - 1],
                                         actual, sizeof(actual)));

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(expected));
}

TEST_CASE("ASIC RX stream resynchronizes after noise and a corrupt frame", "[asic_rx]")
{
    asic_rx_stream_t stream;
    uint8_t corrupt[11];
    uint8_t expected[11];
    uint8_t actual[11] = {0};
    const uint8_t noise[] = {0x00, 0xAA, 0x01, 0x55};
    make_valid_frame(corrupt, sizeof(corrupt), 0x30);
    make_valid_frame(expected, sizeof(expected), 0x40);
    corrupt[4] ^= 0x80;
    asic_rx_stream_reset(&stream);

    for (size_t i = 0; i < sizeof(noise); i++) {
        TEST_ASSERT_FALSE(asic_rx_stream_push(&stream, noise[i], actual, sizeof(actual)));
    }
    for (size_t i = 0; i < sizeof(corrupt); i++) {
        TEST_ASSERT_FALSE(asic_rx_stream_push(&stream, corrupt[i], actual, sizeof(actual)));
    }

    bool complete = false;
    for (size_t i = 0; i < sizeof(expected); i++) {
        complete = asic_rx_stream_push(&stream, expected[i], actual, sizeof(actual));
    }

    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, sizeof(expected));
    TEST_ASSERT_GREATER_THAN_UINT32(0, stream.discarded_bytes);
    TEST_ASSERT_EQUAL_UINT32(1, stream.frames_received);
    TEST_ASSERT_EQUAL_UINT32(1, stream.crc_errors);
}
