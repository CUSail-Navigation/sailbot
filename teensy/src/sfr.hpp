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
        extern uint8_t radio_sail_angle;
        extern uint8_t radio_rudder_angle;
        extern uint8_t ros_sail_angle;
        extern uint8_t ros_rudder_angle;
        extern uint32_t sail_pwm;
        extern uint32_t rudder_pwm;
    }
    namespace serial
    {
        extern bool update_servos_radio;
        extern bool update_servos_ros;
        extern bool send_telemetry;

        extern uint8_t dropped_packets;

        extern uint8_t ros_buffer[2];
        extern uint8_t radio_buffer[3];
        extern uint8_t radio_flag;
    }
}