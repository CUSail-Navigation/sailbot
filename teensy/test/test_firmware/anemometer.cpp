/**
 * ANEMOMETER TESTS -- AnemometerMonitor.
 * This file tests the properties of the AnemometerMonitor that the rest of the boat relies on (including that the
 * reading is a compass bearing, it never wraps past a full circle, and it tracks the sensor in the right direction).
 */
#include "test_support.h"
#include "suites.hpp"
#include "Monitors/AnemometerMonitor.hpp"


// Constants and helper functions.
static constexpr int ADC_MAX = 1023;
static constexpr uint16_t FULL_CIRCLE_DEGREES = 360;

/** Take one reading, with the ADC staged at \code raw\endcode. */
static uint16_t read_wind_angle_for(AnemometerMonitor& monitor, const int raw) {
    mock_set_analog(constants::anemometer::ANEMOMETER_PIN, raw);
    monitor.execute();
    return sfr::anemometer::wind_angle;
}


// Tests.
static void test_zero_reading_is_zero_degrees() {
    AnemometerMonitor monitor;
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(0, read_wind_angle_for(monitor, 0),
                                     "A zero ADC reading should be a zero-degree bearing");
}

static void test_every_reading_stays_within_one_revolution() {
    AnemometerMonitor monitor;

    for (int raw = 0; raw <= ADC_MAX; raw++) {
        const uint16_t angle = read_wind_angle_for(monitor, raw);
        if (angle >= FULL_CIRCLE_DEGREES) {
            char message[128];
            snprintf(message, sizeof(message), "ADC reading %d produced %u degrees, which is not a valid bearing",
                     raw, static_cast<unsigned>(angle));
            TEST_FAIL_MESSAGE(message);
        }
    }
}

static void test_angle_rises_with_sensor_reading() {
    AnemometerMonitor monitor;
    std::vector<uint32_t> sweep;

    for (int raw = 0; raw <= ADC_MAX; raw += 16) sweep.push_back(read_wind_angle_for(monitor, raw));

    assert_non_decreasing(sweep, "wind angle");
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(sweep.front(), sweep.back(),
                                            "A full-scale reading should differ from a zero reading");
}

static void test_full_scale_reading_covers_almost_whole_circle() {
    AnemometerMonitor monitor;
    const uint16_t angle = read_wind_angle_for(monitor, ADC_MAX);

    TEST_ASSERT_LESS_THAN_UINT16(FULL_CIRCLE_DEGREES, angle);
    TEST_ASSERT_GREATER_THAN_UINT16_MESSAGE(FULL_CIRCLE_DEGREES - 5, angle,
                                            "A full-scale reading should map to nearly a full circle");
}

static void test_reading_vane_isolated() {
    AnemometerMonitor monitor;
    read_wind_angle_for(monitor, ADC_MAX / 2);

    TEST_ASSERT_EQUAL_UINT8(0, sfr::serial::dropped_packets);
    TEST_ASSERT_FALSE(sfr::serial::update_servos_usb);
    TEST_ASSERT_FALSE(sfr::serial::update_servos_radio);
    TEST_ASSERT_EQUAL_UINT32(0, sfr::servo::rudder_pwm);
}


// Runner.
void run_anemometer_tests() {
    Unity.TestFile = __FILE__; // Report failures against this file, not main.cpp.
    RUN_TEST(test_zero_reading_is_zero_degrees);
    RUN_TEST(test_every_reading_stays_within_one_revolution);
    RUN_TEST(test_angle_rises_with_sensor_reading);
    RUN_TEST(test_full_scale_reading_covers_almost_whole_circle);
    RUN_TEST(test_reading_vane_isolated);
}
