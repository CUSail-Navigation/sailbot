#include "ServoControlTask.hpp"

ServoControlTask::ServoControlTask()
{
    sail_servo.attach(constants::servo::SAIL_PIN);
    rudder_servo.attach(constants::servo::RUDDER_PIN);
    // Set initial servo positions to 0-degrees
    actuate_servo(sail_servo, 1050);
    actuate_servo(rudder_servo, 1050);
}

void ServoControlTask::execute()
{
    // check if we have new serial data to update servos
    if (sfr::serial::update_servos)
    {
        uint8_t sail_angle = sfr::serial::buffer[0];
        uint8_t rudder_angle = sfr::serial::buffer[1];

        // update sfr values based on incoming serial data if checks pass
        if (sail_angle >= constants::servo::SAIL_MIN_ANGLE && sail_angle <= constants::servo::SAIL_MAX_ANGLE)
        {
            sfr::servo::sail_angle = sail_angle;
            sfr::servo::sail_pwm = sail_to_pwm(sail_angle);

            // actuate sail servo
            actuate_servo(sail_servo, sfr::servo::sail_pwm);
        }

        if (rudder_angle >= constants::servo::RUDDER_MIN_ANGLE && rudder_angle <= constants::servo::RUDDER_MAX_ANGLE)
        {
            sfr::servo::rudder_angle = rudder_angle;
            sfr::servo::rudder_pwm = tail_to_pwm(rudder_angle);

            // actuate rudder servo
            actuate_servo(rudder_servo, sfr::servo::rudder_pwm);
        }
        sfr::serial::update_servos = false; // reset flag for next update
    }
}

// uint32_t ServoControlTask::angle_to_pwm(uint8_t angle)
// {
//     // FIXME: this is hardcoded for our specific scenario
//     return angle * 2;
// }

uint32_t ServoControlTask::tail_to_pwm(uint8_t angle)
{
    return angle * 2;
}

uint32_t ServoControlTask::sail_to_pwm(uint8_t angle)
{
    return map(angle, 0, 90, 1050, 1200);
}

void ServoControlTask::actuate_servo(Servo &servo, uint32_t pwm)
{
    servo.write(pwm);
}