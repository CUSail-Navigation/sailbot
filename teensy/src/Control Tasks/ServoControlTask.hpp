#pragma once
#include <Servo.h>
#include "constants.hpp"
#include "sfr.hpp"

/** NOTES FROM 2025-2026 SEASON: <p>
 * - Both \code mainsail_servo\endcode and \code rudder_servo\endcode have a PWM range of 600-2400 (see constants.hpp).
 * - Mainsail: 600 is "all-in", 2400 is "all-out" (on both sides). This servo can turn 7.85 times.
 * - Rudder: 600 represents "minimum" angle, 2400 represents "maximum" angle. This servo can turn 0.5 times.
 *           Since the rudder goes from -65 degrees to 65 degrees, the corresponding PWM range is 850 - 2150.
 */
class ServoControlTask {
public:
    ServoControlTask();
    void execute();

private:
    Servo rudder_servo;
    Servo mainsail_servo;
    Servo jib_servo1;
    Servo jib_servo2;

    // uint32_t angle_to_pwm(uint8_t angle);
    uint32_t rudder_to_pwm(uint8_t angle);
    uint32_t mainsail_to_pwm(uint8_t angle);
    uint32_t jib1_to_pwm(uint8_t angle);
    uint32_t jib2_to_pwm(uint8_t angle);

    void actuate_servo(Servo &servo, uint32_t pwm);
};