#pragma once
#include "constants.hpp"

/** SFR stands for State Field Registry. It contains all sensor values and
 *  universal flags that should be available to the entire boat. */
namespace sfr {
    namespace anemometer {
        extern uint16_t wind_angle;
    }
    namespace servo {
        extern uint8_t rudder_angle;
        extern uint8_t mainsail_angle;
        extern uint8_t jib_angle;
        extern uint8_t jib_side_flag;

        extern uint32_t rudder_pwm;
        extern uint32_t mainsail_pwm;
        extern uint32_t jib_port_pwm;
        extern uint32_t jib_stb_pwm;
    }
    namespace serial {
        extern bool update_servos_radio;
        extern bool update_servos_ros;
        extern uint8_t dropped_packets;
        extern uint8_t ros_buffer[constants::serial::BUFFER_LEN];
        extern uint8_t radio_buffer[constants::radio::BUFFER_LEN];
        extern uint8_t radio_flag;
    }
}
