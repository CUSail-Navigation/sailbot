#include "sfr.hpp"

namespace sfr {
    namespace anemometer {
        uint16_t wind_angle = 0;
    }
    namespace servo {
        //TODO what do these pre-set values do? Should we change: eg set rudder to 0? port and stb not 0 at the same time?
        uint8_t rudder_angle = 0;
        uint8_t mainsail_angle = 0;
        uint8_t jib_angle = 0;

        //TODO should we make these 600 + rope the constants into here?
        uint32_t rudder_pwm = 0;    // FIXME: is this a PWM value? set to whatever corresponds to angle of 0
        uint32_t mainsail_pwm = 0;  // FIXME: is this a PWM value? set to whatever corresponds to angle of 0
        uint32_t jib_port_pwm = 0;
        uint32_t jib_stb_pwm = 0;
    }
    namespace serial {
        bool update_servos = false;

        uint8_t dropped_packets = 0;

        uint8_t buffer[3] = {0};
    }
}