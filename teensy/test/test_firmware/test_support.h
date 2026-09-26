/**
 * THIS FILE CONTAINS SHARED TEST HELPERS. It accomplishes two main jobs:
 *  1. Reset all scraps of global state between tests to start fresh each time (the SFR and mocks are both global).
 *  2. Build serial packets and pick test angles symbolically -- from constants.hpp, never from literals.
 */
#pragma once
#include <unity.h>
#include "../mocks/Arduino.h"
#include "../mocks/Servo.h"
#include "sfr.hpp"

// Packet layouts: constants for indices defined explicitly (if a format ever gains a field, just make one edit here).
// TODO consider just defining these in constants.hpp (it's a good practice for the rest of the codebase anyway).
namespace layout {
    // USB payload: [mainsail_angle, rudder_angle, jib_angle, jib_side_flag]
    constexpr size_t USB_MAINSAIL  = 0;
    constexpr size_t USB_RUDDER    = 1;
    constexpr size_t USB_JIB       = 2;
    constexpr size_t USB_JIB_SIDE  = 3;

    // Radio payload: [radio_flag, mainsail_angle, rudder_angle, jib_angle, jib_side_flag]
    constexpr size_t RADIO_FLAG      = 0;
    constexpr size_t RADIO_MAINSAIL  = 1;
    constexpr size_t RADIO_RUDDER    = 2;
    constexpr size_t RADIO_JIB       = 3;
    constexpr size_t RADIO_JIB_SIDE  = 4;
}


// State reset.
/** Restore the SFR to the power-on values defined in \code sfr.cpp\endcode. */
inline void reset_sfr() {
    sfr::anemometer::wind_angle = 0;

    sfr::servo::rudder_angle = 0;
    sfr::servo::mainsail_angle = 0;
    sfr::servo::jib_angle = 0;
    sfr::servo::jib_side_flag = 0;
    sfr::servo::rudder_pwm = 0;
    sfr::servo::mainsail_pwm = 0;
    sfr::servo::jib_port_pwm = 0;
    sfr::servo::jib_stb_pwm = 0;

    sfr::serial::update_servos_radio = false;
    sfr::serial::update_servos_usb = false;
    sfr::serial::dropped_packets = 0;
    memset(sfr::serial::usb_buffer, 0, sizeof(sfr::serial::usb_buffer));
    memset(sfr::serial::radio_buffer, 0, sizeof(sfr::serial::radio_buffer));
    sfr::serial::radio_flag = 1;
}

/** Clear the fake clock, both serial ports, and all recorded pin/servo activity. */
inline void reset_mocks() {
    mock_set_millis(0);
    Serial.mock_clear();
    Serial2.mock_clear();
    mock_reset_servos();
    for (size_t pin = 0; pin < MOCK_PIN_COUNT; ++pin) {
        g_mock_pin_modes[pin] = 0;
        g_mock_pin_levels[pin] = 0;
        g_mock_analog_values[pin] = 0;
    }
}

/** Call this function from Unity's \code setUp()\endcode so every test starts from the same baseline. */
inline void reset_all() {
    reset_mocks();
    reset_sfr();
}


// Packet construction.
/** A distinct, deterministic payload byte for slot \code i\endcode that isn't one of the signal flags. */
inline uint8_t safe_payload_byte(const size_t i) {
    uint8_t byte = static_cast<uint8_t>(i + 1);
    while (byte == constants::serial::RX_START_FLAG || byte == constants::serial::RX_END_FLAG) ++byte;
    return byte;
}

/** Returns \code count\endcode distinct, flag-safe payload bytes. */
inline std::vector<uint8_t> make_payload(const size_t count) {
    std::vector<uint8_t> payload;
    payload.reserve(count);
    for (size_t i = 0; i < count; ++i) payload.push_back(safe_payload_byte(i));
    return payload;
}

/** Wrap a payload in the RX framing flags, producing bytes exactly as they arrive over the wire. */
inline std::vector<uint8_t> frame_packet(const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> packet;
    packet.reserve(payload.size() + 2);
    packet.push_back(constants::serial::RX_START_FLAG);
    packet.insert(packet.end(), payload.begin(), payload.end());
    packet.push_back(constants::serial::RX_END_FLAG);
    return packet;
}

/** Queue bytes on a port as if the peer had just sent them. */
inline void feed(FakeStream& port, const std::vector<uint8_t>& bytes) {
    port.mock_rx(bytes.data(), bytes.size());
}


// Angle calculation helper methods, derived from limits that constants.hpp currently declares.
/** The midpoint of an inclusive angle range. */
inline uint8_t mid_angle(const uint8_t lo, const uint8_t hi) {
    return static_cast<uint8_t>(lo + (hi - lo) / 2);
}

/**
 * Finds an angle guaranteed to fall outside the inclusive range [lo, hi], for testing bounds rejection.
 * Returns \code false\endcode when the range covers the whole \code uint8_t\endcode domain and no invalid value
 * exists (the caller should then skip the test rather than assert something impossible).
 */
inline bool find_out_of_range_angle(const uint8_t lo, const uint8_t hi, uint8_t& out) {
    if (hi < 255) { out = static_cast<uint8_t>(hi + 1); return true; }
    if (lo > 0)   { out = static_cast<uint8_t>(lo - 1); return true; }
    return false;
}

/** Generates an invalid jib side flag (neither port nor stb). */
inline bool find_invalid_jib_side_flag(uint8_t& out) {
    for (unsigned candidate = 0; candidate <= 255; ++candidate) {
        const uint8_t flag = static_cast<uint8_t>(candidate);
        if (flag != constants::servo::JIB_SIDE_PORT && flag != constants::servo::JIB_SIDE_STB) {
            out = flag;
            return true;
        }
    }
    return false;
}

/** Asserts that every step of a sweep goes up. Used for the rudder, whose \code map()\endcode is linear with no
 *  clamping. */
inline void assert_strictly_increasing(const std::vector<uint32_t>& values, const char* what) {
    for (size_t i = 1; i < values.size(); ++i) {
        if (values[i] <= values[i - 1]) {
            char message[192];
            snprintf(message, sizeof(message),
                     "%s should rise with angle, but step %zu went %lu -> %lu",
                     what, i, static_cast<unsigned long>(values[i - 1]), static_cast<unsigned long>(values[i]));
            TEST_FAIL_MESSAGE(message);
        }
    }
}

/**
 * Asserts that a sweep never goes down. Used for the sails, whose law-of-cosines mapping rises but then flattens once
 * it clamps at \code MAX_PULSE\endcode, so consecutive values are allowed to be equal but never to reverse.
 */
inline void assert_non_decreasing(const std::vector<uint32_t>& values, const char* what) {
    for (size_t i = 1; i < values.size(); ++i) {
        if (values[i] < values[i - 1]) {
            char message[192];
            snprintf(message, sizeof(message),
                     "%s should never fall as angle rises, but step %zu went %lu -> %lu",
                     what, i, static_cast<unsigned long>(values[i - 1]), static_cast<unsigned long>(values[i]));
            TEST_FAIL_MESSAGE(message);
        }
    }
}

/** Asserts that a PWM value sits inside the servo's declared pulse range. */
inline void assert_pwm_in_range(const uint32_t pwm, const uint32_t min_pulse, const uint32_t max_pulse,
                                const char* what) {
    if (pwm < min_pulse || pwm > max_pulse) {
        char message[160];
        snprintf(message, sizeof(message), "%s PWM %lu outside [%lu, %lu]",
                 what, static_cast<unsigned long>(pwm),
                 static_cast<unsigned long>(min_pulse), static_cast<unsigned long>(max_pulse));
        TEST_FAIL_MESSAGE(message);
    }
}
