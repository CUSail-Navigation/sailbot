#include "TrackerControlTask.hpp"

TrackerControlTask::TrackerControlTask()
{
    tracker_servo.attach(constants::servo::TRACKER_PIN);
}

void TrackerControlTask::execute()
{
    if (sfr::serial::update_servos)
    {
        if (sfr::serial::buoy_displacement > 0)
        {
            actuate_servo(tracker_servo, read_servo(tracker_servo) + 10);
        }
        else if (sfr::serial::buoy_displacement < 0)
        {
            actuate_servo(tracker_servo, read_servo(tracker_servo) - 10);
        }
        {
            actuate_servo(tracker_servo, read_servo(tracker_servo));
            sfr::serial::buoy_angle = read_servo(tracker_servo);
        }
    }
}

uint32_t TrackerControlTask::angle_to_pwm(uint8_t angle)
{
    return map(angle, 0, 90, 1050, 1300); // TODO: find out what these numbers mean
}

void TrackerControlTask::actuate_servo(Servo &servo, uint32_t pwm)
{
    servo.write(pwm);
}

uint8_t TrackerControlTask::read_servo(Servo &servo)
{
    return servo.read();
}