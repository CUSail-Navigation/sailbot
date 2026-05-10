#include "ServoControlTask.hpp"

ServoControlTask::ServoControlTask()
{
    sail_servo.attach(constants::servo::SAIL_PIN, constants::servo::SAIL_MIN_PULSE, constants::servo::SAIL_MAX_PULSE);
    rudder_servo.attach(constants::servo::RUDDER_PIN);
    // Set initial servo positions to 0-degrees
    actuate_servo(sail_servo, 0);
    actuate_servo(rudder_servo, 1050);
}

void ServoControlTask::execute()
{
    if (sfr::serial::radio_flag != 0)
    {
        // RC mode: drop radio updates if link has gone silent
        if (millis() - sfr::serial::last_radio_packet_ms > constants::serial::RADIO_TIMEOUT_MS)
        {
            sfr::serial::update_servos_radio = false;
            return;
        }

        if (!sfr::serial::update_servos_radio)
            return;

        uint8_t sail_angle   = sfr::serial::radio_buffer[1];
        uint8_t rudder_angle = sfr::serial::radio_buffer[2];

        if (sail_angle >= constants::servo::SAIL_MIN_ANGLE && sail_angle <= constants::servo::SAIL_MAX_ANGLE)
        {
            sfr::servo::radio_sail_angle = sail_angle;
            sfr::servo::sail_pwm = sail_to_pwm(sail_angle);
            actuate_servo(sail_servo, sfr::servo::sail_pwm);
        }
        if (rudder_angle >= constants::servo::RUDDER_MIN_ANGLE && rudder_angle <= constants::servo::RUDDER_MAX_ANGLE)
        {
            sfr::servo::radio_rudder_angle = rudder_angle;
            sfr::servo::rudder_pwm = tail_to_pwm(rudder_angle);
            actuate_servo(rudder_servo, sfr::servo::rudder_pwm);
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
            actuate_servo(sail_servo, sfr::servo::sail_pwm);
        }
        if (rudder_angle >= constants::servo::RUDDER_MIN_ANGLE && rudder_angle <= constants::servo::RUDDER_MAX_ANGLE)
        {
            sfr::servo::ros_rudder_angle = rudder_angle;
            sfr::servo::rudder_pwm = tail_to_pwm(rudder_angle);
            actuate_servo(rudder_servo, sfr::servo::rudder_pwm);
        }

        sfr::serial::update_servos_ros = false;
    }
}

uint32_t ServoControlTask::tail_to_pwm(uint8_t angle)
{
    return map(angle, 0, 50, 45, 55);
}

uint32_t ServoControlTask::sail_to_pwm(uint8_t angle)
{
    // return map(angle, 0, 90, 1050, 1200);
    return map(angle, 0, 90, 38, 0);
}

void ServoControlTask::actuate_servo(Servo &servo, uint32_t pwm)
{
    servo.write(pwm);
}