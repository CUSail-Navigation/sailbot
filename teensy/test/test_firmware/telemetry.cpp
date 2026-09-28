/**
 * TELEMETRY TESTS -- TelemetryControlTask.
 * The tests in this file assert the format of the telemetry packet and the send cadence. The byte order is a protocol
 * contract shared with the Jetson, so it is tested explicitly; everything else comes from the SFR/constants.hpp.
 */
#include "test_support.h"
#include "suites.hpp"
#include "ControlTasks/TelemetryControlTask.hpp"


// Helper functions.
/**
 * \code TelemetryControlTask\endcode compares against the timestamp captured at the END of the previous
 * \code execute()\endcode, so a freshly constructed task needs two calls before the first frame goes out.
 * This lag is harmless in the real loop (runs continuously) but must be reproduced here to observe any telemetry.
 */
static void init_telemetry(TelemetryControlTask& task) {
    task.execute();
    task.execute();
}

/** The frame the firmware should produce for whatever is currently in the SFR. */
static std::vector<uint8_t> expected_frame() {
    return {
        constants::serial::TX_START_FLAG,
        static_cast<uint8_t>(sfr::anemometer::wind_angle >> 8),
        static_cast<uint8_t>(sfr::anemometer::wind_angle & 0xFF),
        sfr::servo::mainsail_angle,
        sfr::servo::rudder_angle,
        sfr::servo::jib_angle,
        sfr::servo::jib_side_flag,
        sfr::serial::dropped_packets,
        constants::serial::TX_END_FLAG,
    };
}


// Tests.
static void test_nothing_is_sent_before_the_period_elapses() {
    TelemetryControlTask task;

    mock_set_millis(constants::serial::TX_PERIOD_MS - 1);
    init_telemetry(task);

    TEST_ASSERT_EQUAL_size_t_MESSAGE(0, Serial.mock_tx().size(), "Telemetry must not send before TX_PERIOD_MS has passed");
}

static void test_frame_layout_matches_the_protocol() {
    // Values chosen purely as test input.
    sfr::anemometer::wind_angle = 345;
    sfr::servo::mainsail_angle = 12;
    sfr::servo::rudder_angle = 34;
    sfr::servo::jib_angle = 56;
    sfr::servo::jib_side_flag = constants::servo::JIB_SIDE_STB;
    sfr::serial::dropped_packets = 7;

    TelemetryControlTask task;
    mock_set_millis(constants::serial::TX_PERIOD_MS);
    init_telemetry(task);

    const std::vector<uint8_t> expected = expected_frame();
    const std::vector<uint8_t>& actual = Serial.mock_tx();

    TEST_ASSERT_EQUAL_size_t_MESSAGE(expected.size(), actual.size(), "Telemetry frame is the wrong length");
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected.data(), actual.data(), expected.size());
}

static void test_wind_angle_is_split_big_endian() {
    sfr::anemometer::wind_angle = 347; // 0x015B: high byte 0x01, low byte 0x5B.

    TelemetryControlTask task;
    mock_set_millis(constants::serial::TX_PERIOD_MS);
    init_telemetry(task);

    const std::vector<uint8_t>& frame = Serial.mock_tx();
    TEST_ASSERT_GREATER_THAN_size_t(2, frame.size());
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0x01, frame[1], "Wind angle high byte should come first");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0x5B, frame[2], "Wind angle low byte should come second");

    // A wind angle above 255 is exactly why this field is two bytes; make sure it survives the round trip.
    const uint16_t reassembled = static_cast<uint16_t>((frame[1] << 8) | frame[2]);
    TEST_ASSERT_EQUAL_UINT16(sfr::anemometer::wind_angle, reassembled);
}

static void test_frame_is_not_repeated_until_another_period_passes() {
    TelemetryControlTask task;
    mock_set_millis(constants::serial::TX_PERIOD_MS);
    init_telemetry(task);

    const size_t after_first = Serial.mock_tx().size();
    TEST_ASSERT_GREATER_THAN_size_t_MESSAGE(0, after_first, "The first frame should have been sent by now");

    // Another loop iteration immediately afterwards must not produce a second frame.
    task.execute();
    TEST_ASSERT_EQUAL_size_t_MESSAGE(after_first, Serial.mock_tx().size(),
                                     "Telemetry should be rate limited, not sent every loop iteration");

    // Once a full period has passed, the next frame goes out.
    mock_advance_millis(constants::serial::TX_PERIOD_MS);
    task.execute();
    task.execute();
    TEST_ASSERT_GREATER_THAN_size_t_MESSAGE(after_first, Serial.mock_tx().size(),
                                            "A second frame should follow one TX_PERIOD_MS later");
}

static void test_dropped_packet_count_is_reported() {
    sfr::serial::dropped_packets = 42;

    TelemetryControlTask task;
    mock_set_millis(constants::serial::TX_PERIOD_MS);
    init_telemetry(task);

    const std::vector<uint8_t>& frame = Serial.mock_tx();
    TEST_ASSERT_EQUAL_size_t(expected_frame().size(), frame.size());
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(42, frame[frame.size() - 2], "Dropped counter belongs just before the end flag");
}


// Runner.
void run_telemetry_tests() {
    Unity.TestFile = __FILE__; // Report failures against this file, not main.cpp.
    RUN_TEST(test_nothing_is_sent_before_the_period_elapses);
    RUN_TEST(test_frame_layout_matches_the_protocol);
    RUN_TEST(test_wind_angle_is_split_big_endian);
    RUN_TEST(test_frame_is_not_repeated_until_another_period_passes);
    RUN_TEST(test_dropped_packet_count_is_reported);
}
