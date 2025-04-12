#include "ServoControlTask.hpp"
#include <math.h>
#include <math.h>

ServoControlTask::ServoControlTask()
{
    sail_servo.attach(constants::servo::SAIL_PIN);
    rudder_servo.attach(constants::servo::RUDDER_PIN);

    // Set initial servo positions to 0-degrees
    actuate_servo(sail_servo, 1350);
    actuate_servo(rudder_servo, rudder_to_pwm(0));
}

void ServoControlTask::execute()
{
    // check if we have new serial data to update servos
    if (sfr::serial::update_servos)
    {
        uint8_t sail_angle = sfr::serial::buffer[0];
        uint8_t rudder_angle = sfr::serial::buffer[1];

        if(sfr::serial::autonomous_mode)
        {
            uint8_t auto_sail_angle = trim_sail();
            sfr::servo::sail_angle = auto_sail_angle;
            sfr::servo::sail_pwm = sail_to_pwm(auto_sail_angle);
            // actuate sail servo
            actuate_servo(sail_servo, sfr::servo::sail_pwm);
            // Serial.println("In autonomous mode");
        }
        else
        {
            // update sfr values based on incoming serial data if it checks pass
            if (sail_angle >= constants::servo::SAIL_MIN_ANGLE && sail_angle <= constants::servo::SAIL_MAX_ANGLE)
            {
                sfr::servo::sail_angle = sail_angle;
                sfr::servo::sail_pwm = sail_to_pwm(sail_angle);

                // actuate sail servo
                actuate_servo(sail_servo, sfr::servo::sail_pwm);
                // Serial.println("In RC sail mode");
            }
        }

        if (sfr::servo::rudder_angle >= constants::servo::RUDDER_MIN_ANGLE && sfr::servo::rudder_angle <= constants::servo::RUDDER_MAX_ANGLE)
        {
            sfr::servo::rudder_pwm = rudder_to_pwm(sfr::servo::rudder_angle);
            Serial.println(sfr::servo::rudder_angle);

            // actuate rudder servo
            actuate_servo(rudder_servo, sfr::servo::rudder_pwm);
            // Serial.println("In rudder");
        }
        sfr::serial::update_servos = false; // reset flag for next update
    }
}

uint8_t ServoControlTask::trim_sail()
{
    uint16_t normal_wind_angle = sfr::anemometer::wind_angle;
    normal_wind_angle = 180 - abs(normal_wind_angle - 180); //Normalized wind angle to 0 - 180 always because we only care about one side
    
    if ((normal_wind_angle >= 0 && normal_wind_angle <= 10))
    {
        return 0;
    }
    else if (normal_wind_angle >= 150)
    {
        return 90;
    }
    else
    {
        return map(normal_wind_angle, 10, 150, 0, 90);
    }
}

uint32_t ServoControlTask::rudder_to_pwm(int8_t angle)
{
    return map(angle, -30, 30, 1325, 1675);
}

uint32_t ServoControlTask::sail_to_pwm(uint8_t angle)
{
    return map(angle, 0, 90, 1350, 1050);
}

void ServoControlTask::actuate_servo(Servo &servo, uint32_t pwm)
{
    servo.write(pwm);
}
