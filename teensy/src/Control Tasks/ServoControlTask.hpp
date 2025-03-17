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
    uint32_t sail_to_pwm(uint8_t angle);
    uint32_t rudder_to_pwm(uint8_t angle);

    uint8_t trim_sail();
    
    void actuate_servo(Servo &servo, uint32_t pwm);
    
    Servo sail_servo;
    Servo rudder_servo;

    
};