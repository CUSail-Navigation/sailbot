#pragma once
#include <Servo.h>
#include "constants.hpp"
#include "sfr.hpp"

/** NOTES FROM 2025-2026 SEASON: <p>
 * - Rudder: 600 represents "minimum" angle, 2400 represents "maximum" angle. This servo can turn 0.5 times.
 *           Since the rudder goes from -65 degrees to 65 degrees, the corresponding PWM range is 850 - 2150.
 * - Mainsail: 600 is "all-in", 2400 is "all-out" (on both sides). This servo can turn 7.85 times.
 */
class ServoControlTask {
public:
    ServoControlTask();
    void execute();

private:
    Servo rudder_servo;
    Servo mainsail_servo;
    Servo jib_port_servo;
    Servo jib_stb_servo;

    uint32_t rudder_to_pwm(uint8_t angle);
    uint32_t mainsail_to_pwm(uint8_t angle);
    uint32_t jib_port_to_pwm(uint8_t angle);
    uint32_t jib_stb_to_pwm(uint8_t angle);

    void actuate_servo(Servo &servo, uint32_t pwm);
};