#include "TrackerControlTask.hpp"

TrackerControlTask::TrackerControlTask()
{
    tracker_servo.attach(constants::servo::TRACKER_PIN);
}

void TrackerControlTask::execute()
{
    // Serial.println(sfr::serial::update_servos);

    if (sfr::serial::update_servos)
    {
        Serial.println("SDJFOIWEJFOIWEJFOEWIJF");
        // Directly set servo to the commanded angle
        actuate_servo(tracker_servo, angle_to_pwm(sfr::serial::servo_angle));
        
        // Update the buoy angle to reflect the current servo position
        sfr::serial::buoy_angle = sfr::serial::servo_angle;
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