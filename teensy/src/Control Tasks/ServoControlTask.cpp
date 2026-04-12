#include "ServoControlTask.hpp"

/** Both servos have a PWM range of 600 - 2400 (see constants.hpp).  <p>
 * NOTES FROM 2025-2026 SEASON:
 * - Main sail: 600 is "all-in", 2400 is "all-out" (on both sides). This servo can turn 7.85 times.
 * - Rudder: 600 represents "minimum" angle, 2400 represents "maximum" angle. This servo can turn 0.5 times.
 *           Since the rudder goes from -65 degrees to 65 degrees, the corresponding PWM range is 850 - 2150.
 */
ServoControlTask::ServoControlTask() {
    sail_servo.attach(constants::servo::SAIL_PIN, constants::servo::SAIL_MIN_PULSE, constants::servo::SAIL_MAX_PULSE);
    rudder_servo.attach(constants::servo::RUDDER_PIN, constants::servo::RUDDER_MIN_PULSE, constants::servo::RUDDER_MID_PULSE, constants::servo::RUDDER_MAX_PULSE); //todo what does this do? but since I added the last 3 constants I put them here

    // (2025-2026) Pre-set the sail to "all-in", rudder to center.
    actuate_servo(sail_servo, constants::servo::SAIL_MIN_PULSE);
    actuate_servo(rudder_servo, constants::servo::RUDDER_MID_PULSE);
}

void ServoControlTask::execute() {
    // Check if we have new serial data to update the servos.
    if (sfr::serial::update_servos) {
        uint8_t sail_angle = sfr::serial::buffer[0];
        uint8_t rudder_angle = sfr::serial::buffer[1];

        // Update sfr values based on incoming serial data if checks pass.
        if (sail_angle >= constants::servo::SAIL_MIN_ANGLE && sail_angle <= constants::servo::SAIL_MAX_ANGLE) {
            sfr::servo::sail_angle = sail_angle;
            sfr::servo::sail_pwm = sail_to_pwm(sail_angle);

            actuate_servo(sail_servo, sfr::servo::sail_pwm);
        }
        if (rudder_angle >= constants::servo::RUDDER_MIN_ANGLE && rudder_angle <= constants::servo::RUDDER_MAX_ANGLE) {
            sfr::servo::rudder_angle = rudder_angle;
            sfr::servo::rudder_pwm = tail_to_pwm(rudder_angle);

            actuate_servo(rudder_servo, sfr::servo::rudder_pwm);
        }

        sfr::serial::update_servos = false; // Reset flag for the next update.
    }
}

/** Maps a goal rudder angle to a servo PWM.
 *
 * @param angle the goal angle to set the rudder to.
 * @return the PWM to actuate the servo to.
 */
uint32_t ServoControlTask::tail_to_pwm(uint8_t angle) { //todo why is this called tail_to_pwm? --> change to rudder_to_pwm?
    return map(angle, constants::servo::RUDDER_MIN_ANGLE, constants::servo::RUDDER_MAX_ANGLE, 850, 2150);
}

/** Maps a goal sail angle to a servo PWM.
 *
 * @param angle the goal angle to set the sail to.
 * @return the PWM to actuate the servo to.
 */
uint32_t ServoControlTask::sail_to_pwm(uint8_t angle) {
    // Use the law of cosines (get length of mainsheet relative to "all-in").
    // In this case, we have c = sqrt(b^2 + b^2 - 2*b*b*cos(angle) where b is the length of boom.
    uint8_t b = 1;                                                                                                      //todo find real value
    float mainsheet_len = sqrtf( 2 * b * b * (1 - cosf(angle * 0.017453)) ); // 0.017453 = pi/180.

    // Final PWM = (PWM_per_turn * num_turns + base_PWM) where num_turns = mainsheet_len / servo_wheel_circumference.
    return (uint32_t) ( 229.3f * (mainsheet_len / 12.927f) + 600 );                                                     //todo consider adding some offsets. consider adding constants to file
}

void ServoControlTask::actuate_servo(Servo &servo, uint32_t pwm) {
    servo.write(pwm);
}