/**
 * SERVO CONTROL TESTS -- ServoControlTask.
 * Tests in this file cover two main areas of functionality:
 *  1. Mode arbitration (radio serial must win over USB serial whenever radio_flag != 0, a fresh radio or USB command
 *     applies and clears its own update flag, an idle cycle with nothing pending commands no servo at all).
 *  2. Angle to PWM mapping.
 */
#include "test_support.h"
#include "suites.hpp"
#include "ControlTasks/ServoControlTask.hpp"


static_assert(constants::serial::USB_BUFFER_LEN > layout::USB_JIB_SIDE, "usb_buffer is too small for the documented USB layout");
static_assert(constants::serial::RADIO_BUFFER_LEN > layout::RADIO_JIB_SIDE, "radio_buffer is too small for the documented radio layout");


// Helper functions.
/** Angles that are valid under any calibration, for the fields a test is not focused on. */
static uint8_t default_mainsail() {
    return mid_angle(constants::servo::MAINSAIL_MIN_ANGLE, constants::servo::MAINSAIL_MAX_ANGLE);
}
static uint8_t default_rudder() {
    return mid_angle(constants::servo::RUDDER_MIN_ANGLE, constants::servo::RUDDER_MAX_ANGLE);
}
static uint8_t default_jib() {
    return mid_angle(constants::servo::JIB_MIN_ANGLE, constants::servo::JIB_MAX_ANGLE);
}

/** Stage a USB command in the SFR exactly as \code USBSerialMonitor\endcode would, and select USB mode. */
static void stage_usb_command(const uint8_t mainsail, const uint8_t rudder, const uint8_t jib, const uint8_t jib_side) {
    sfr::serial::usb_buffer[layout::USB_MAINSAIL] = mainsail;
    sfr::serial::usb_buffer[layout::USB_RUDDER] = rudder;
    sfr::serial::usb_buffer[layout::USB_JIB] = jib;
    sfr::serial::usb_buffer[layout::USB_JIB_SIDE] = jib_side;
    sfr::serial::update_servos_usb = true;
    sfr::serial::radio_flag = 0;
}

/** Stage a radio command in the SFR exactly as \code RadioSerialMonitor\endcode would, and select radio mode. */
static void stage_radio_command(const uint8_t mainsail, const uint8_t rudder, const uint8_t jib, const uint8_t jib_side) {
    sfr::serial::radio_buffer[layout::RADIO_FLAG] = 1;
    sfr::serial::radio_buffer[layout::RADIO_MAINSAIL] = mainsail;
    sfr::serial::radio_buffer[layout::RADIO_RUDDER] = rudder;
    sfr::serial::radio_buffer[layout::RADIO_JIB] = jib;
    sfr::serial::radio_buffer[layout::RADIO_JIB_SIDE] = jib_side;
    sfr::serial::update_servos_radio = true;
    sfr::serial::radio_flag = 1;
}

/** Run one USB command through the task and return, so a sweep can read the resulting PWM out of the SFR. */
static void apply_usb_command(ServoControlTask& task, const uint8_t mainsail, const uint8_t rudder,
                              const uint8_t jib, const uint8_t jib_side) {
    stage_usb_command(mainsail, rudder, jib, jib_side);
    task.execute();
}


// Mode arbitration tests.
static void test_radio_mode_ignores_pending_usb_command() {
    ServoControlTask task;
    mock_reset_servos();

    // Have a USB packet pending, but radio mode is engaged. USB packet shouldn't process.
    stage_usb_command(default_mainsail(), default_rudder(), default_jib(), constants::servo::JIB_SIDE_PORT);
    sfr::serial::radio_flag = 1;
    sfr::serial::update_servos_radio = false;
    task.execute();

    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, sfr::servo::mainsail_angle,
                                    "A queued USB command must not reach the servos while radio mode is engaged");
    TEST_ASSERT_TRUE_MESSAGE(sfr::serial::update_servos_usb,
                             "The USB command should remain pending, not be silently consumed");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, g_mock_servo_write_count[constants::servo::RUDDER_PIN],
                                  "No servo should be commanded at all on this cycle");
}

static void test_radio_mode_applies_a_fresh_radio_command() {
    ServoControlTask task;
    const uint8_t mainsail = default_mainsail();
    const uint8_t rudder = default_rudder();
    const uint8_t jib = default_jib();

    stage_radio_command(mainsail, rudder, jib, constants::servo::JIB_SIDE_PORT);
    task.execute();

    TEST_ASSERT_EQUAL_UINT8(mainsail, sfr::servo::mainsail_angle);
    TEST_ASSERT_EQUAL_UINT8(rudder, sfr::servo::rudder_angle);
    TEST_ASSERT_EQUAL_UINT8(jib, sfr::servo::jib_angle);
    TEST_ASSERT_FALSE_MESSAGE(sfr::serial::update_servos_radio, "The radio flag should be cleared once consumed");
}

static void test_usb_mode_applies_when_radio_flag_is_zero() {
    ServoControlTask task;
    const uint8_t mainsail = default_mainsail();
    const uint8_t rudder = default_rudder();

    apply_usb_command(task, mainsail, rudder, default_jib(), constants::servo::JIB_SIDE_PORT);

    TEST_ASSERT_EQUAL_UINT8(mainsail, sfr::servo::mainsail_angle);
    TEST_ASSERT_EQUAL_UINT8(rudder, sfr::servo::rudder_angle);
    TEST_ASSERT_FALSE_MESSAGE(sfr::serial::update_servos_usb, "The USB flag should be cleared once consumed");
}

static void test_no_pending_command_moves_nothing() {
    ServoControlTask task;
    mock_reset_servos();

    sfr::serial::radio_flag = 0;
    sfr::serial::update_servos_usb = false;
    sfr::serial::update_servos_radio = false;
    task.execute();

    TEST_ASSERT_EQUAL_INT_MESSAGE(0, g_mock_servo_write_count[constants::servo::MAINSAIL_PIN],
                                  "An idle cycle should not command any servo");
}


// Tests for input validation: out-of-range values must be discarded.
static void test_out_of_range_mainsail_angle_is_rejected() {
    uint8_t bad_angle;
    if (!find_out_of_range_angle(constants::servo::MAINSAIL_MIN_ANGLE, constants::servo::MAINSAIL_MAX_ANGLE, bad_angle)) {
        TEST_IGNORE_MESSAGE("Mainsail angle limits span the whole byte range; no invalid value exists to test");
    }

    ServoControlTask task;
    apply_usb_command(task, bad_angle, default_rudder(), default_jib(), constants::servo::JIB_SIDE_PORT);

    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, sfr::servo::mainsail_angle, "An out-of-range mainsail angle must be discarded");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(default_rudder(), sfr::servo::rudder_angle,
                                    "A bad mainsail angle must not block the valid rudder angle in the same packet");
}

static void test_out_of_range_rudder_angle_is_rejected() {
    uint8_t bad_angle;
    if (!find_out_of_range_angle(constants::servo::RUDDER_MIN_ANGLE, constants::servo::RUDDER_MAX_ANGLE, bad_angle)) {
        TEST_IGNORE_MESSAGE("Rudder angle limits span the whole byte range; no invalid value exists to test");
    }

    ServoControlTask task;
    apply_usb_command(task, default_mainsail(), bad_angle, default_jib(), constants::servo::JIB_SIDE_PORT);

    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, sfr::servo::rudder_angle, "An out-of-range rudder angle must be discarded");
}

static void test_out_of_range_jib_angle_is_rejected() {
    uint8_t bad_angle;
    if (!find_out_of_range_angle(constants::servo::JIB_MIN_ANGLE, constants::servo::JIB_MAX_ANGLE, bad_angle)) {
        TEST_IGNORE_MESSAGE("Jib angle limits span the whole byte range; no invalid value exists to test");
    }

    ServoControlTask task;
    apply_usb_command(task, default_mainsail(), default_rudder(), bad_angle, constants::servo::JIB_SIDE_PORT);

    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, sfr::servo::jib_angle, "An out-of-range jib angle must be discarded");
}

static void test_invalid_jib_side_flag_is_rejected() {
    uint8_t bad_flag;
    if (!find_invalid_jib_side_flag(bad_flag)) TEST_IGNORE_MESSAGE("Every byte value is a valid jib side flag");

    ServoControlTask task;
    apply_usb_command(task, default_mainsail(), default_rudder(), default_jib(), bad_flag);

    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, sfr::servo::jib_angle, "Corrupt jib side flag means whole jib command discard");
}


// Tests for rudder mapping.
static void test_rudder_endpoints_match_the_declared_pulse_range() {
    ServoControlTask task;

    apply_usb_command(task, default_mainsail(), constants::servo::RUDDER_MIN_ANGLE, default_jib(),
                      constants::servo::JIB_SIDE_PORT);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(constants::servo::RUDDER_MIN_PULSE, sfr::servo::rudder_pwm,
                                     "The smallest rudder angle should map to RUDDER_MIN_PULSE");

    apply_usb_command(task, default_mainsail(), constants::servo::RUDDER_MAX_ANGLE, default_jib(),
                      constants::servo::JIB_SIDE_PORT);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(constants::servo::RUDDER_MAX_PULSE, sfr::servo::rudder_pwm,
                                     "The largest rudder angle should map to RUDDER_MAX_PULSE");
}

static void test_rudder_midpoint_is_amidships() {
    ServoControlTask task;
    apply_usb_command(task, default_mainsail(), default_rudder(), default_jib(), constants::servo::JIB_SIDE_PORT);

    TEST_ASSERT_UINT32_WITHIN_MESSAGE(1, constants::servo::RUDDER_MID_PULSE, sfr::servo::rudder_pwm,
                                      "A mid-range rudder angle should sit at RUDDER_MID_PULSE");
}

static void test_rudder_pwm_rises_with_angle_and_stays_in_range() {
    ServoControlTask task;
    std::vector<uint32_t> sweep;

    for (int angle = constants::servo::RUDDER_MIN_ANGLE; angle <= constants::servo::RUDDER_MAX_ANGLE; ++angle) {
        apply_usb_command(task, default_mainsail(), static_cast<uint8_t>(angle), default_jib(),
                          constants::servo::JIB_SIDE_PORT);
        assert_pwm_in_range(sfr::servo::rudder_pwm, constants::servo::RUDDER_MIN_PULSE,
                            constants::servo::RUDDER_MAX_PULSE, "rudder");
        sweep.push_back(sfr::servo::rudder_pwm);
    }

    assert_strictly_increasing(sweep, "rudder");
}


// Test for sail mappings (law-of-cosines based: exact values are not predictable, but the shape of the curve is).
static void test_mainsail_pwm_never_falls_and_stays_in_range() {
    ServoControlTask task;
    std::vector<uint32_t> sweep;

    for (int angle = constants::servo::MAINSAIL_MIN_ANGLE; angle <= constants::servo::MAINSAIL_MAX_ANGLE; ++angle) {
        apply_usb_command(task, static_cast<uint8_t>(angle), default_rudder(), default_jib(),
                          constants::servo::JIB_SIDE_PORT);
        assert_pwm_in_range(sfr::servo::mainsail_pwm, constants::servo::MAINSAIL_MIN_PULSE,
                            constants::servo::MAINSAIL_MAX_PULSE, "mainsail");
        sweep.push_back(sfr::servo::mainsail_pwm);
    }

    assert_non_decreasing(sweep, "mainsail");
    TEST_ASSERT_GREATER_THAN_UINT32_MESSAGE(sweep.front(), sweep.back(),
                                            "Sheeting the mainsail all the way out should differ from all the way in");
}

static void test_mainsail_minimum_angle_sits_at_minimum_pulse() {
    if constexpr (constants::servo::MAINSAIL_MIN_ANGLE != 0) {
        TEST_IGNORE_MESSAGE("This anchor only holds when the minimum mainsail angle is 0 degrees");
    }

    ServoControlTask task;
    apply_usb_command(task, constants::servo::MAINSAIL_MIN_ANGLE, default_rudder(), default_jib(),
                      constants::servo::JIB_SIDE_PORT);

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(constants::servo::MAINSAIL_MIN_PULSE, sfr::servo::mainsail_pwm,
                                     "Zero mainsail angle means zero sheet paid out, i.e. MAINSAIL_MIN_PULSE");
}

static void test_jib_pwm_never_falls_and_stays_in_range_on_both_sides() {
    ServoControlTask task;

    for (const uint8_t side : {constants::servo::JIB_SIDE_PORT, constants::servo::JIB_SIDE_STB}) {
        const bool is_port = (side == constants::servo::JIB_SIDE_PORT);
        const uint32_t min_pulse = is_port ? constants::servo::JIB_PORT_MIN_PULSE : constants::servo::JIB_STB_MIN_PULSE;
        const uint32_t max_pulse = is_port ? constants::servo::JIB_PORT_MAX_PULSE : constants::servo::JIB_STB_MAX_PULSE;

        std::vector<uint32_t> sweep;
        for (int angle = constants::servo::JIB_MIN_ANGLE; angle <= constants::servo::JIB_MAX_ANGLE; ++angle) {
            apply_usb_command(task, default_mainsail(), default_rudder(), static_cast<uint8_t>(angle), side);
            const uint32_t pwm = is_port ? sfr::servo::jib_port_pwm : sfr::servo::jib_stb_pwm;
            assert_pwm_in_range(pwm, min_pulse, max_pulse, is_port ? "jib (port)" : "jib (starboard)");
            sweep.push_back(pwm);
        }
        assert_non_decreasing(sweep, is_port ? "jib (port)" : "jib (starboard)");
    }
}

static void test_trimming_port_jib_slacks_the_starboard_sheet() {
    ServoControlTask task;
    apply_usb_command(task, default_mainsail(), default_rudder(), default_jib(), constants::servo::JIB_SIDE_PORT);

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(constants::servo::JIB_STB_MAX_PULSE, sfr::servo::jib_stb_pwm,
                                     "Trimming to port should let the starboard sheet all the way out");
    assert_pwm_in_range(sfr::servo::jib_port_pwm, constants::servo::JIB_PORT_MIN_PULSE,
                        constants::servo::JIB_PORT_MAX_PULSE, "jib (port)");
    TEST_ASSERT_EQUAL_UINT8(constants::servo::JIB_SIDE_PORT, sfr::servo::jib_side_flag);
}

static void test_trimming_starboard_jib_slacks_the_port_sheet() {
    ServoControlTask task;
    apply_usb_command(task, default_mainsail(), default_rudder(), default_jib(), constants::servo::JIB_SIDE_STB);

    TEST_ASSERT_EQUAL_UINT32_MESSAGE(constants::servo::JIB_PORT_MAX_PULSE, sfr::servo::jib_port_pwm,
                                     "Trimming to starboard should let the port sheet all the way out");
    assert_pwm_in_range(sfr::servo::jib_stb_pwm, constants::servo::JIB_STB_MIN_PULSE,
                        constants::servo::JIB_STB_MAX_PULSE, "jib (starboard)");
    TEST_ASSERT_EQUAL_UINT8(constants::servo::JIB_SIDE_STB, sfr::servo::jib_side_flag);
}


// Simulates wiring: the computed PWM must actually reach the servo on the right pin.
static void test_computed_pwm_reaches_the_correct_servo_pins() {
    ServoControlTask task;
    mock_reset_servos();
    apply_usb_command(task, default_mainsail(), default_rudder(), default_jib(), constants::servo::JIB_SIDE_PORT);

    TEST_ASSERT_EQUAL_INT_MESSAGE(static_cast<int>(sfr::servo::rudder_pwm),
                                  g_mock_servo_last_write[constants::servo::RUDDER_PIN],
                                  "The rudder servo should receive the PWM recorded in the SFR");
    TEST_ASSERT_EQUAL_INT_MESSAGE(static_cast<int>(sfr::servo::mainsail_pwm),
                                  g_mock_servo_last_write[constants::servo::MAINSAIL_PIN],
                                  "The mainsail servo should receive the PWM recorded in the SFR");
    TEST_ASSERT_EQUAL_INT_MESSAGE(static_cast<int>(sfr::servo::jib_port_pwm),
                                  g_mock_servo_last_write[constants::servo::JIB_PORT_PIN],
                                  "The port jib servo should receive the PWM recorded in the SFR");
}


// Runner.
void run_servo_control_tests() {
    Unity.TestFile = __FILE__; // Report failures against this file, not main.cpp.
    RUN_TEST(test_radio_mode_ignores_pending_usb_command);
    RUN_TEST(test_radio_mode_applies_a_fresh_radio_command);
    RUN_TEST(test_usb_mode_applies_when_radio_flag_is_zero);
    RUN_TEST(test_no_pending_command_moves_nothing);

    RUN_TEST(test_out_of_range_mainsail_angle_is_rejected);
    RUN_TEST(test_out_of_range_rudder_angle_is_rejected);
    RUN_TEST(test_out_of_range_jib_angle_is_rejected);
    RUN_TEST(test_invalid_jib_side_flag_is_rejected);

    RUN_TEST(test_rudder_endpoints_match_the_declared_pulse_range);
    RUN_TEST(test_rudder_midpoint_is_amidships);
    RUN_TEST(test_rudder_pwm_rises_with_angle_and_stays_in_range);

    RUN_TEST(test_mainsail_pwm_never_falls_and_stays_in_range);
    RUN_TEST(test_mainsail_minimum_angle_sits_at_minimum_pulse);
    RUN_TEST(test_jib_pwm_never_falls_and_stays_in_range_on_both_sides);
    RUN_TEST(test_trimming_port_jib_slacks_the_starboard_sheet);
    RUN_TEST(test_trimming_starboard_jib_slacks_the_port_sheet);

    RUN_TEST(test_computed_pwm_reaches_the_correct_servo_pins);
}
