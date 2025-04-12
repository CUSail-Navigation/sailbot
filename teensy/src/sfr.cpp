#include "sfr.hpp"

namespace sfr
{
    namespace anemometer
    {
        uint16_t wind_angle = 0;
    }
    namespace servo
    {
        uint8_t sail_angle = 0;
        uint8_t rudder_angle = 0;
        uint32_t sail_pwm = 0;
        uint32_t rudder_pwm = 0;
    }
    namespace serial
    {
        bool update_servos = false;

        uint8_t dropped_packets = 0;

        uint8_t buffer[3] = {0};
    }
}