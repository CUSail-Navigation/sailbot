#include "ServoControlTask.hpp"

/**
 *  Construct a \code ServoControlTask\endcode and initialize all servos to default values.
 */
ServoControlTask::ServoControlTask() {
    rudder_servo.attach(constants::servo::RUDDER_PIN, constants::servo::RUDDER_MIN_PULSE, constants::servo::RUDDER_MAX_PULSE);
    mainsail_servo.attach(constants::servo::MAINSAIL_PIN, constants::servo::MAINSAIL_MIN_PULSE, constants::servo::MAINSAIL_MAX_PULSE);
    jib_port_servo.attach(constants::servo::JIB_PORT_PIN, constants::servo::JIB_PORT_MIN_PULSE, constants::servo::JIB_PORT_MAX_PULSE);
    jib_stb_servo.attach(constants::servo::JIB_STB_PIN, constants::servo::JIB_STB_MIN_PULSE, constants::servo::JIB_STB_MAX_PULSE);

    // Pre-set the rudder to center and all sail servos to "all-out" (for manual setup).
    actuate_servo(rudder_servo, constants::servo::RUDDER_MID_PULSE);
    actuate_servo(mainsail_servo, constants::servo::MAINSAIL_MAX_PULSE);
    actuate_servo(jib_port_servo, constants::servo::JIB_PORT_MAX_PULSE);
    actuate_servo(jib_stb_servo, constants::servo::JIB_STB_MAX_PULSE);
ServoControlTask::ServoControlTask()
{
    sail_servo.attach(constants::servo::SAIL_PIN, constants::servo::SAIL_MIN_PULSE, constants::servo::SAIL_MAX_PULSE);
    rudder_servo.attach(constants::servo::RUDDER_PIN, 500, 2500);
    sail_servo.writeMicroseconds(constants::servo::SAIL_MAX_PULSE);
    rudder_servo.writeMicroseconds(1500);  // start centered
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

        // Update sfr values and actuate servos based on incoming serial data, if checks pass.
        if (rudder_angle >= constants::servo::RUDDER_MIN_ANGLE && rudder_angle <= constants::servo::RUDDER_MAX_ANGLE) {
            sfr::servo::rudder_angle = rudder_angle;
            sfr::servo::rudder_pwm = rudder_to_pwm(rudder_angle);
            actuate_servo(rudder_servo, sfr::servo::rudder_pwm);
void ServoControlTask::execute()
{
    if (sfr::serial::radio_flag != 0)
    {
        if (!sfr::serial::update_servos_radio)
            return;

        uint8_t sail_angle   = sfr::serial::radio_buffer[1];
        uint8_t rudder_angle = sfr::serial::radio_buffer[2];

        if (sail_angle >= constants::servo::SAIL_MIN_ANGLE && sail_angle <= constants::servo::SAIL_MAX_ANGLE)
        {
            sfr::servo::radio_sail_angle = sail_angle;
            sfr::servo::sail_pwm = sail_to_pwm(sail_angle);
            sail_servo.writeMicroseconds(sfr::servo::sail_pwm);
        }
        if (rudder_angle >= constants::servo::RUDDER_MIN_ANGLE && rudder_angle <= constants::servo::RUDDER_MAX_ANGLE)
        {
            sfr::servo::radio_rudder_angle = rudder_angle;
            sfr::servo::rudder_pwm = tail_to_pwm(rudder_angle);
            rudder_servo.writeMicroseconds(sfr::servo::rudder_pwm);
        }

        sfr::serial::update_servos_radio = false;
    }
    else
    {
        // Jetson mode
        if (!sfr::serial::update_servos_ros)
            return;

        uint8_t sail_angle   = sfr::serial::ros_buffer[0];
        uint8_t rudder_angle = sfr::serial::ros_buffer[1];

        if (sail_angle >= constants::servo::SAIL_MIN_ANGLE && sail_angle <= constants::servo::SAIL_MAX_ANGLE)
        {
            sfr::servo::ros_sail_angle = sail_angle;
            sfr::servo::sail_pwm = sail_to_pwm(sail_angle);
            sail_servo.writeMicroseconds(sfr::servo::sail_pwm);
        }
        if (rudder_angle >= constants::servo::RUDDER_MIN_ANGLE && rudder_angle <= constants::servo::RUDDER_MAX_ANGLE)
        {
            sfr::servo::ros_rudder_angle = rudder_angle;
            sfr::servo::rudder_pwm = tail_to_pwm(rudder_angle);
            rudder_servo.writeMicroseconds(sfr::servo::rudder_pwm);
        }
        if (mainsail_angle >= constants::servo::MAINSAIL_MIN_ANGLE &&
            mainsail_angle <= constants::servo::MAINSAIL_MAX_ANGLE) {
            sfr::servo::mainsail_angle = mainsail_angle;
            sfr::servo::mainsail_pwm = mainsail_to_pwm(mainsail_angle);
            actuate_servo(mainsail_servo, sfr::servo::mainsail_pwm);
        }
        if (jib_angle >= constants::servo::JIB_MIN_ANGLE && jib_angle <= constants::servo::JIB_MAX_ANGLE &&
            (jib_side_flag == constants::servo::JIB_SIDE_PORT || jib_side_flag == constants::servo::JIB_SIDE_STB)) {
            sfr::servo::jib_angle = jib_angle;
            sfr::servo::jib_side_flag = jib_side_flag;

            // Decide PWMs for both sides and actuate. Always actuate the "loose" side first.
            if (jib_side_flag == constants::servo::JIB_SIDE_PORT) {
                sfr::servo::jib_stb_pwm = constants::servo::JIB_STB_MAX_PULSE;
                actuate_servo(jib_stb_servo, constants::servo::JIB_STB_MAX_PULSE);
                sfr::servo::jib_port_pwm = jib_to_pwm(jib_angle, jib_side_flag);
                actuate_servo(jib_port_servo, sfr::servo::jib_port_pwm);
            }
            else {
                sfr::servo::jib_port_pwm = constants::servo::JIB_PORT_MAX_PULSE;
                actuate_servo(jib_port_servo, constants::servo::JIB_PORT_MAX_PULSE);
                sfr::servo::jib_stb_pwm = jib_to_pwm(jib_angle, jib_side_flag);
                actuate_servo(jib_stb_servo, sfr::servo::jib_stb_pwm);
            }
        }

        sfr::serial::update_servos = false; // Reset flag for the next update.

        sfr::serial::update_servos_ros = false;
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
uint32_t ServoControlTask::tail_to_pwm(uint8_t angle)
{
    // angle is offset-encoded: 0 = -45°, 45 = 0° (center), 90 = +45°
    return map(angle, constants::servo::RUDDER_MIN_ANGLE, constants::servo::RUDDER_MAX_ANGLE,
               constants::servo::RUDDER_MIN_PULSE, constants::servo::RUDDER_MAX_PULSE);
}

/** Maps a goal mainsail angle to a \code mainsail_servo\endcode PWM.
 *
 * @param angle the goal angle to set the mainsail to.
 * @return the PWM to actuate \code mainsail_servo\endcode to.
 */
uint32_t ServoControlTask::mainsail_to_pwm(const uint8_t angle) {
    return law_of_cos_map(angle,
                          constants::servo::TWO_BOOM_LEN_SQD_CM,
                          constants::servo::MAINSAIL_PULSE_PER_TURN,
                          constants::servo::MAINSAIL_WHEEL_CIRCUM_CM,
                          true
    ) + constants::servo::MAINSAIL_MIN_PULSE;
}

/** Maps a goal sail angle for the jib to a PWM value (for one of the two jib servos).
 *
 * @param angle the goal angle to set the jib to (on the appropriate side of the boat).
 * @param jib_side_flag which servo we are calculating a PWM value for (which side we are setting the jib to).
 * @return the PWM to actuate the appropriate servo to.
 */
uint32_t ServoControlTask::jib_to_pwm(const uint8_t angle, const uint8_t jib_side_flag) {
    const float wheel_circum = (jib_side_flag == constants::servo::JIB_SIDE_PORT) ?
                                constants::servo::JIB_PORT_WHEEL_CIRCUM_CM : constants::servo::JIB_STB_WHEEL_CIRCUM_CM;
    const uint32_t min_pulse = (jib_side_flag == constants::servo::JIB_SIDE_PORT) ?
                                constants::servo::JIB_PORT_MIN_PULSE : constants::servo::JIB_STB_MIN_PULSE;
    return law_of_cos_map(angle,
                          constants::servo::TWO_JIB_FOOT_LEN_SQD_CM,
                          constants::servo::JIB_PULSE_PER_TURN,
                          wheel_circum,
                          false
    ) + min_pulse;
uint32_t ServoControlTask::sail_to_pwm(uint8_t angle)
{
    return map(angle, constants::servo::SAIL_MIN_ANGLE, constants::servo::SAIL_MAX_ANGLE,
               constants::servo::SAIL_MAX_PULSE, constants::servo::SAIL_MIN_PULSE);
}

/** Send \code pwm\endcode to \code servo\endcode, thereby changing \code servo\endcode 's angle. */
void ServoControlTask::actuate_servo(Servo &servo, const uint32_t pwm) {
    servo.write(static_cast<int>(pwm));
}

/** A utility method that uses the law of cosines to map a goal sail angle to a servo PWM value.
 *  Used for both the mainsail servo and the jib servos.
 *
 *  Extended explanation as follows:
 * - The law of cosines states \code c^2 = a^2 + b^2 - 2ab*cos(angle)\endcode
 * - In an isosceles triangle where a = b, and "angle" is the angle between these two legs, this simplifies to
 *   \code c^2 = 2b^2(1 - cos(angle))\endcode
 * - Consider the mainsail, namely looking down at the boom from high above the boat:
 *    - "angle" refers to the angle that the boom makes with the centerline of the boat; namely, the goal angle we want
 *      the mainsail to be set to.
 *    - Pivoting the boom around the mast traces out an isosceles triangle: the length of the legs ("b") is the length
 *      of the section of the boom from the mast to where the mainsheet attaches.
 *    - "c" is the component of the mainsheet length in the plane that the boom rotates in.
 *    - We must take the initial length of the mainsheet, \code MAINSAIL_INITIAL_CM\endcode, into account; we therefore
 *      apply the Pythagorean theorem on the right triangle formed by "c", the initial distance from the deck to the
 *      boom, and the actual mainsheet, to obtain the actual goal length of the mainsheet.
 * - We model similarly for the jib (note that this is less accurate as there is no "boom" for the jib, and it is
 *   therefore harder to quantify a triangle or angles in general).
 *    - "angle" refers to the goal angle we want the jib to be set to.
 *    - We approximate "b" as the length of the foot of the jib.
 *    - Here, "c" on its own is approximately the length of the sheet, which is what we want to solve for.
 * - In either case, our final \code sheet_len\endcode value maps to a PWM value based on the specific servo and
 *   the circumference of its wheel.
 *
 * Note that this method assumes that the max PWM value of the servo in question allows for the sail to be set to
 * \code angle\endcode. If this is not the case, it will return a value greater than the servo's PWM range.
 */
uint32_t ServoControlTask::law_of_cos_map(const uint8_t angle, const uint32_t two_b_sqd, const float PWM_per_turn,
                                          const float wheel_circum, const bool mainsail) {
    const float c_squared = static_cast<float>(two_b_sqd) * (1 - cosf(static_cast<float>(angle) * 0.017453f)); // 0.017453 = pi/180.
    const float sheet_len = mainsail ? sqrtf(c_squared + constants::servo::MAINSAIL_INITIAL_SQD_CM) - constants::servo::MAINSAIL_INITIAL_CM
                                     : sqrtf(c_squared);

    return static_cast<uint32_t>(PWM_per_turn * (sheet_len / wheel_circum));  // PWM: (PWM_per_turn * turns_needed).
}