#pragma once
#include <Servo.h>
#include "sfr.hpp"

class ServoControlTask {
public:
    ServoControlTask();
    void execute();

private:
    Servo rudder_servo;
    Servo mainsail_servo;
    Servo jib_port_servo;
    Servo jib_stb_servo;

    static uint32_t rudder_to_pwm(uint8_t angle);
    static uint32_t mainsail_to_pwm(uint8_t angle);
    static uint32_t jib_to_pwm(uint8_t angle, uint8_t jib_side_flag);
    static void actuate_servo(Servo &servo, uint32_t pwm);

    static uint32_t law_of_cos_map(uint8_t angle, uint32_t two_b_sqd, float PWM_per_turn, float wheel_circum, bool mainsail);
    void apply_commands(uint8_t mainsail_angle, uint8_t rudder_angle, uint8_t jib_angle, uint8_t jib_side_flag);
};