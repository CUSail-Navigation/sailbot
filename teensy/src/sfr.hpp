#pragma once

#include <Arduino.h>

// SFR stands for State Field Registry. It contains all sensor values, and
// universal flags that should be available to the entire boat.

namespace sfr
{
    namespace anemometer
    {
        extern uint16_t wind_angle;
    }
    namespace servo
    {
        extern uint8_t sail_angle;
        extern int8_t rudder_angle;
        extern uint32_t sail_pwm;
        extern uint32_t rudder_pwm;
    }
    namespace serial
    {
        extern bool update_servos;
        extern bool send_telemetry;
        extern bool autonomous_mode;
        
        extern uint8_t dropped_packets;

        extern uint8_t buffer[3];
    }
}