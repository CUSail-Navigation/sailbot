#include "ServoControlTask.hpp"

/**
 *  Construct a \code ServoControlTask\endcode and initialize all servos to default values.
 */
ServoControlTask::ServoControlTask() {
    rudder_servo.attach(constants::servo::RUDDER_PIN, constants::servo::RUDDER_MIN_PULSE, constants::servo::RUDDER_MAX_PULSE);
    mainsail_servo.attach(constants::servo::MAINSAIL_PIN, constants::servo::MAINSAIL_MIN_PULSE, constants::servo::MAINSAIL_MAX_PULSE);
    jib_port_servo.attach(constants::servo::JIB_PORT_PIN, constants::servo::JIB_PORT_MIN_PULSE, constants::servo::JIB_PORT_MAX_PULSE);
    jib_stb_servo.attach(constants::servo::JIB_STB_PIN, constants::servo::JIB_STB_MIN_PULSE, constants::servo::JIB_STB_MAX_PULSE);

    // (2025-2026) Pre-set the rudder to center, mainsail to "all-in", and jib to "all-in" on the port side.
    actuate_servo(rudder_servo, constants::servo::RUDDER_MID_PULSE);
    actuate_servo(mainsail_servo, constants::servo::MAINSAIL_MIN_PULSE);
    actuate_servo(jib_port_servo, constants::servo::JIB_PORT_MIN_PULSE);
    actuate_servo(jib_stb_servo, constants::servo::JIB_STB_MAX_PULSE);
}

/**
 *  Updates the servos if valid serial data is ready and waiting.
 */
void ServoControlTask::execute() {
    // Check if we have new serial data to update the servos.
    if (sfr::serial::update_servos) {
        const uint8_t mainsail_angle = sfr::serial::buffer[0];
        const uint8_t rudder_angle = sfr::serial::buffer[1];
        const uint8_t jib_port_angle = sfr::serial::buffer[2];
        const uint8_t jib_stb_angle = sfr::serial::buffer[3];

        // Update sfr values based on incoming serial data if checks pass.
        if (rudder_angle >= constants::servo::RUDDER_MIN_ANGLE && rudder_angle <= constants::servo::RUDDER_MAX_ANGLE) {
            sfr::servo::rudder_angle = rudder_angle;
            sfr::servo::rudder_pwm = rudder_to_pwm(rudder_angle);
            actuate_servo(rudder_servo, sfr::servo::rudder_pwm);
        }
        if (mainsail_angle >= constants::servo::MAINSAIL_MIN_ANGLE &&
            mainsail_angle <= constants::servo::MAINSAIL_MAX_ANGLE) {
            sfr::servo::mainsail_angle = mainsail_angle;
            sfr::servo::mainsail_pwm = mainsail_to_pwm(mainsail_angle);
            actuate_servo(mainsail_servo, sfr::servo::mainsail_pwm);
        }
        // (Temporary) Store jib command bytes from serial so we can plumb data end-to-end.
        sfr::servo::jib_port_angle = jib_port_angle;
        sfr::servo::jib_stb_angle = jib_stb_angle;
        // TODO(jib-logic): Validate jib angles against constants::servo::JIB_MIN_ANGLE / JIB_MAX_ANGLE.
        // TODO(jib-logic): Compute and set sfr::servo::jib_port_pwm, then actuate jib_port_servo.
        // TODO(jib-logic): Compute and set sfr::servo::jib_stb_pwm, then actuate jib_stb_servo.
        // TODO(jib-logic): Enforce linked behavior so only one jib side is active at a time.

        sfr::serial::update_servos = false; // Reset flag for the next update.
    }
}

/** Maps a goal rudder angle to a \code rudder_servo\endcode PWM.
 *
 * @param angle the goal angle to set the rudder to.
 * @return the PWM to actuate \code rudder_servo\endcode to.
 */
uint32_t ServoControlTask::rudder_to_pwm(uint8_t angle) {
    return map(angle, constants::servo::RUDDER_MIN_ANGLE, constants::servo::RUDDER_MAX_ANGLE,
               constants::servo::RUDDER_MIN_PULSE, constants::servo::RUDDER_MAX_PULSE);
}

/** Maps a goal mainsail angle to a \code mainsail_servo\endcode PWM.
 *
 * @param angle the goal angle to set the mainsail to.
 * @return the PWM to actuate \code mainsail_servo\endcode to.
 */
uint32_t ServoControlTask::mainsail_to_pwm(uint8_t angle) {
    // Use the law of cosines to get length of mainsheet relative to "all-in", where b is the length of the boom.
    // In this case, we have mainsheet_len = sqrt(b^2 + b^2 - 2*b*b*cos(angle)) = sqrt(2b^2*(1 - cos(angle))).
    float mainsheet_len = sqrtf( 2 * constants::servo::BOOM_LENGTH_CM_SQUARED * (1 - cosf(angle * 0.017453f)) ); // 0.017453 = pi/180.

    // Final PWM = (PWM_per_turn * num_turns + base_PWM) where num_turns = mainsheet_len / servo_wheel_circumference.
    return static_cast<uint32_t>(229.3f * (mainsheet_len / 12.927f) + constants::servo::MAINSAIL_MIN_PULSE);            //TODO consider adding some offsets + consider adding constants to file
}

/** Maps a goal sail angle for the jib, on the port side, to a \code jib_port_servo\endcode PWM.
 *
 * @param angle the goal angle to set the jib to on the port side of the boat.
 * @return the PWM to actuate \code jib_port_servo\endcode to.
 */
uint32_t ServoControlTask::jib_port_to_pwm(uint8_t angle) {
    return map(angle, constants::servo::JIB_MIN_ANGLE, constants::servo::JIB_MAX_ANGLE,
               constants::servo::JIB_PORT_MIN_PULSE, constants::servo::JIB_PORT_MAX_PULSE);
}

/** Maps a goal sail angle for the jib, on the starboard side, to a \code jib_stb_servo\endcode PWM.
 *
 * @param angle the goal angle to set the jib to on the starboard side of the boat.
 * @return the PWM to actuate \code jib_stb_servo\endcode to.
 */
uint32_t ServoControlTask::jib_stb_to_pwm(uint8_t angle) {
    return map(angle, constants::servo::JIB_MIN_ANGLE, constants::servo::JIB_MAX_ANGLE,
               constants::servo::JIB_STB_MIN_PULSE, constants::servo::JIB_STB_MAX_PULSE);
}

/** Send \code pwm\endcode to \code servo\endcode, thereby changing \code servo\endcode 's angle. */
void ServoControlTask::actuate_servo(Servo &servo, const uint32_t pwm) {
    servo.write(pwm);
}