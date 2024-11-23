#include "TrackerControlTask.hpp"

TrackerControlTask::TrackerControlTask()
{
    tracker_servo.attach(constants::servo::TRACKER_PIN);
}

void TrackerControlTask::execute()
{
    if (sfr::serial::update_servos)
    {
        if( sfr::buoy_displacement > 0)
        {
            actuate_servo(tracker_servo, angle_to_pwm(constants::servo::TRACKER_MAX_ANGLE));
        }
        else 
        {
            actuate_servo(tracker_servo, angle_to_pwm(constants::servo::TRACKER_MIN_ANGLE));
        }
    }
}


uint32_t ServoControlTask::angle_to_pwm(uint8_t angle)
{
    return map(angle, 0, 90, 1050, 1300); // TODO: find out what these numbers mean
}

void ServoControlTask::actuate_servo(Servo &servo, uint32_t pwm)
{
    servo.write(pwm);
}