#pragma once
#include <Servo.h>

#include "constants.hpp"
#include "sfr.hpp"

class ServoControlTask
{
public:
    ServoControlTask();
    void execute();

private:
    // uint32_t angle_to_pwm(uint8_t angle);
    uint32_t sail_to_pwm(uint8_t angle);
    uint32_t tail_to_pwm(uint8_t angle);
    uint32_t tracker_to_pwm(uint8_t angle);

    
    void actuate_servo(Servo &servo, uint32_t pwm);

    Servo sail_servo;
    Servo rudder_servo;
    Servo tracker_servo;

    
};