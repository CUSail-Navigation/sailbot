#pragma once

#include <Arduino.h>

// SFR stands for State Field Registry. It contains all sensor values, and
// universal flags that should be available to the entire boat.

namespace sfr
{
    namespace anemometer
    {
        extern uint8_t wind_angle;
    }
    namespace servo
    {
        extern uint8_t sail_angle;
        extern uint8_t rudder_angle;
        extern uint32_t sail_pwm;
        extern uint32_t rudder_pwm;
    }
    namespace serial
    {
        extern bool update_servos;
        extern bool send_telemetry;
        extern int8_t buoy_displacement;

        extern uint8_t dropped_packets;

        extern uint8_t buffer[2];
    }
}