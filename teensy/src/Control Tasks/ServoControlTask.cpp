#include "ServoControlTask.hpp"

/**
 *  Construct a ServoControlTask and initialize all servos to default values.
 */
ServoControlTask::ServoControlTask() {
    mainsail_servo.attach(constants::servo::MAINSAIL_PIN, constants::servo::MAINSAIL_MIN_PULSE, constants::servo::MAINSAIL_MAX_PULSE);
    rudder_servo.attach(constants::servo::RUDDER_PIN, constants::servo::RUDDER_MIN_PULSE, constants::servo::RUDDER_MAX_PULSE);

    // (2025-2026) Pre-set the sail to "all-in", and rudder to center.
    actuate_servo(mainsail_servo, constants::servo::MAINSAIL_MIN_PULSE);
    actuate_servo(rudder_servo, constants::servo::RUDDER_MID_PULSE);
}

/**
 *  Updates the servos if valid serial data is ready and waiting.
 */
void ServoControlTask::execute() {
    // Check if we have new serial data to update the servos.
    if (sfr::serial::update_servos) {
        uint8_t mainsail_angle = sfr::serial::buffer[0];
        uint8_t rudder_angle = sfr::serial::buffer[1];

        // Update sfr values based on incoming serial data if checks pass.
        if (mainsail_angle >= constants::servo::MAINSAIL_MIN_ANGLE &&
            mainsail_angle <= constants::servo::MAINSAIL_MAX_ANGLE) {
            sfr::servo::mainsail_angle = mainsail_angle;
            sfr::servo::mainsail_pwm = mainsail_to_pwm(mainsail_angle);

            actuate_servo(mainsail_servo, sfr::servo::mainsail_pwm);
        }
        if (rudder_angle >= constants::servo::RUDDER_MIN_ANGLE && rudder_angle <= constants::servo::RUDDER_MAX_ANGLE) {
            sfr::servo::rudder_angle = rudder_angle;
            sfr::servo::rudder_pwm = rudder_to_pwm(rudder_angle);

            actuate_servo(rudder_servo, sfr::servo::rudder_pwm);
        }

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
 * @param angle the goal angle to set the sail to.
 * @return the PWM to actuate \code mainsail_servo\endcode to.
 */
uint32_t ServoControlTask::mainsail_to_pwm(uint8_t angle) {
    // Use the law of cosines (get length of mainsheet relative to "all-in").
    // In this case, we have c = sqrt(b^2 + b^2 - 2*b*b*cos(angle)) where b is the length of boom (cm).
    uint8_t b = 1;                                                                                                      //todo find real value
    float mainsheet_len = sqrtf( 2 * b * b * (1 - cosf(angle * 0.017453)) ); // 0.017453 = pi/180.

    // Final PWM = (PWM_per_turn * num_turns + base_PWM) where num_turns = mainsheet_len / servo_wheel_circumference.
    return (uint32_t) ( 229.3f * (mainsheet_len / 12.927f) + constants::servo::MAINSAIL_MIN_PULSE );                    //todo consider adding some offsets + consider adding constants to file
}

//TODO: for both jib servos: must initialize in ServoControlTask(), add update logic to in execute(), and add constants for in constants.hpp.
/** Maps a goal sail angle for the jib, on the port side, to a \code jib_servo1\endcode PWM.
 *
 * @param angle the goal angle to set the jib to on the PORT side of the boat.
 * @return the PWM to actuate \code jib_servo1\endcode to.
 */
uint32_t jib1_to_pwm(uint8_t angle) {
    return 0;
}

/** Maps a goal sail angle for the jib, on the starboard side, to a \code jib_servo2\endcode PWM.
 *
 * @param angle the goal angle to set the jib to on the PORT side of the boat.
 * @return the PWM to actuate \code jib_servo2\endcode to.
 */
uint32_t jib2_to_pwm(uint8_t angle) {
    return 0;
}

/** Send \code pwm\endcode to \code servo\endcode, thereby changing its angle. */
void ServoControlTask::actuate_servo(Servo &servo, uint32_t pwm) {
    servo.write(pwm);
}