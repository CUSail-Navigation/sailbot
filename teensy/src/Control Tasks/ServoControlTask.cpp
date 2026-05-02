#include "ServoControlTask.hpp"

/**
 *  Construct a \code ServoControlTask\endcode and initialize all servos to default values.
 */
ServoControlTask::ServoControlTask() {
    rudder_servo.attach(constants::servo::RUDDER_PIN, constants::servo::RUDDER_MIN_PULSE, constants::servo::RUDDER_MAX_PULSE);
    mainsail_servo.attach(constants::servo::MAINSAIL_PIN, constants::servo::MAINSAIL_MIN_PULSE, constants::servo::MAINSAIL_MAX_PULSE);
    jib_port_servo.attach(constants::servo::JIB_PORT_PIN, constants::servo::JIB_PORT_MIN_PULSE, constants::servo::JIB_PORT_MAX_PULSE);
    jib_stb_servo.attach(constants::servo::JIB_STB_PIN, constants::servo::JIB_STB_MIN_PULSE, constants::servo::JIB_STB_MAX_PULSE);

    // Pre-set the rudder to center, mainsail to "all-in", and jib to "all-in" on the port side.
    actuate_servo(rudder_servo, constants::servo::RUDDER_MID_PULSE);
    actuate_servo(mainsail_servo, constants::servo::MAINSAIL_MIN_PULSE);
    actuate_servo(jib_stb_servo, constants::servo::JIB_STB_MAX_PULSE);
    actuate_servo(jib_port_servo, constants::servo::JIB_PORT_MIN_PULSE);
}

/**
 *  Updates the servos if valid serial data is ready and waiting.
 */
void ServoControlTask::execute() {
    // Check if we have new serial data to update the servos.
    if (sfr::serial::update_servos) {
        const uint8_t mainsail_angle = sfr::serial::buffer[0];
        const uint8_t rudder_angle = sfr::serial::buffer[1];
        const uint8_t jib_angle = sfr::serial::buffer[2];
        const uint8_t jib_side_flag = sfr::serial::buffer[3];

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
        if (true) { // TODO(jib-logic): Validate jib_angle against constants::servo::JIB_MIN_ANGLE / JIB_MAX_ANGLE.
            sfr::servo::jib_angle = jib_angle;
            sfr::servo::jib_side_flag = jib_side_flag;
            // TODO(jib-logic): Use jib_side_flag to choose port vs starboard sheet; compute jib_port_pwm / jib_stb_pwm; actuate.
        }

        sfr::serial::update_servos = false; // Reset flag for the next update.
    }
}

/** Maps a goal rudder angle to a \code rudder_servo\endcode PWM.
 *
 * @param angle the goal angle to set the rudder to.
 * @return the PWM to actuate \code rudder_servo\endcode to.
 */
uint32_t ServoControlTask::rudder_to_pwm(const uint8_t angle) {
    return map(angle, constants::servo::RUDDER_MIN_ANGLE, constants::servo::RUDDER_MAX_ANGLE,
               constants::servo::RUDDER_MIN_PULSE, constants::servo::RUDDER_MAX_PULSE);
}

/** Maps a goal mainsail angle to a \code mainsail_servo\endcode PWM.
 *
 * @param angle the goal angle to set the mainsail to.
 * @return the PWM to actuate \code mainsail_servo\endcode to.
 */
uint32_t ServoControlTask::mainsail_to_pwm(const uint8_t angle) {
    return lawOfCosines( angle,
                         constants::servo::TWO_BOOM_LEN_SQD_CM,
                         constants::servo::MAINSAIL_PWM_PER_TURN,
                         constants::servo::MAINSAIL_WHEEL_CIRCUM_CM
    ) + constants::servo::MAINSAIL_MIN_PULSE;
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
    servo.write(static_cast<int>(pwm));
}

/** A utility method to map a goal sail angle to a servo PWM value.
 *  Used for both the mainsail servo and the jib servos.
 *
 *  Extended explanation as follows:
 * - The law of cosines states c = sqrt{a^2 + b^2 - 2abcos(angle)}.
 * - In an isoceles triangle where a = b, and "angle" is the angle between these two legs, this simplifies to
 *   c = sqrt{2b^2(1 - cos(angle))}
 * - Consider the mainsail, namely looking at the boom from high above the boat:
 *    - "angle" refers to the angle that the boom makes with the centerline of the boat; namely the goal angle we want
 *      the sail to be set to.
 *    - Pivoting the boom around mast traces out an isoceles triangle: the length of the legs ("b") are the section of
 *      the boom from the mast to where it connects to the mainsheet.
 * - We model similarly for the jib, although this is more difficult as there is no "boom" for the jib and therefore
 *   it's harder to quantify a triangle.
 *    - "angle" still refers to the goal angle we want the sail to be set to.
 *    - We approximate "b" as the length of the foot of the jib.
 * - In either case, "c" is approximately the length of the sheet, which is what we want to solve for.
 * - "c" maps to some PWM value based on the specific servo and the circumference of its wheel.
 */
uint32_t ServoControlTask::lawOfCosines(const uint8_t angle, const uint32_t two_b_sqd, const float PWM_per_turn, const float wheel_circum) {
    const float c = sqrtf( two_b_sqd * (1 - cosf(angle * 0.017453f)) ); // 0.017453 = pi/180.
    return static_cast<uint32_t>(PWM_per_turn * (c / wheel_circum));    // PWM value: (PWM_per_turn * turns_needed).
}