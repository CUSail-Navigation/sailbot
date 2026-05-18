#include "ServoControlTask.hpp"

ServoControlTask::ServoControlTask()
{
    sail_servo.attach(constants::servo::SAIL_PIN, constants::servo::SAIL_MIN_PULSE, constants::servo::SAIL_MAX_PULSE);
    rudder_servo.attach(constants::servo::RUDDER_PIN, 500, 2500);
    sail_servo.writeMicroseconds(constants::servo::SAIL_MAX_PULSE);
    rudder_servo.writeMicroseconds(1500);  // start centered
}

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

        sfr::serial::update_servos_ros = false;
    }
}

uint32_t ServoControlTask::tail_to_pwm(uint8_t angle)
{
    // angle is offset-encoded: 0 = -45°, 45 = 0° (center), 90 = +45°
    return map(angle, constants::servo::RUDDER_MIN_ANGLE, constants::servo::RUDDER_MAX_ANGLE,
               constants::servo::RUDDER_MIN_PULSE, constants::servo::RUDDER_MAX_PULSE);
}

uint32_t ServoControlTask::sail_to_pwm(uint8_t angle)
{
    return map(angle, constants::servo::SAIL_MIN_ANGLE, constants::servo::SAIL_MAX_ANGLE,
               constants::servo::SAIL_MAX_PULSE, constants::servo::SAIL_MIN_PULSE);
}

void ServoControlTask::actuate_servo(Servo &servo, uint32_t pwm)
{
    servo.write(pwm);
}