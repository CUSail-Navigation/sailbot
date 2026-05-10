#include "sfr.hpp"

namespace sfr
{
    namespace anemometer
    {
        uint16_t wind_angle = 0;
    }
    namespace servo
    {
        uint8_t ros_sail_angle = 0;
        uint8_t ros_rudder_angle = 0;
        uint8_t radio_sail_angle = 0;
        uint8_t radio_rudder_angle = 0;
        uint32_t sail_pwm = 0;
        uint32_t rudder_pwm = 0;
    }
    namespace serial
    {
        bool update_servos_radio = false;
        bool update_servos_ros = false;

        uint8_t dropped_packets = 0;

        uint8_t ros_buffer[2] = {0};
        uint8_t radio_buffer[3] = {0};
        uint8_t radio_flag = 1;
    }
}