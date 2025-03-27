#pragma once
#include <Servo.h>

#include "constants.hpp"
#include "sfr.hpp"

class TrackerControlTask
{
public:
    TrackerControlTask();
    void execute();

private:
    uint32_t angle_to_pwm(uint8_t angle);
    void actuate_servo(Servo &servo, uint32_t pwm);
    uint8_t read_servo(Servo &servo);

    Servo tracker_servo;
};