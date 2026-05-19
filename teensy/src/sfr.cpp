#include "sfr.hpp"

/** Initialize all fields here; these values will be changed immediately when relevant. */
namespace sfr {
    namespace anemometer {
        uint16_t wind_angle = 0;
    }
    namespace servo {
        uint8_t rudder_angle = 0;
        uint8_t mainsail_angle = 0;
        uint8_t jib_angle = 0;
        uint8_t jib_side_flag = 0;

        uint32_t rudder_pwm = 0;
        uint32_t mainsail_pwm = 0;
        uint32_t jib_port_pwm = 0;
        uint32_t jib_stb_pwm = 0;

        uint8_t radio_sail_angle = 0;
        uint8_t radio_rudder_angle = 0;
        uint8_t ros_sail_angle = 0;
        uint8_t ros_rudder_angle = 0;
    }
    namespace serial {
        bool update_servos_radio = false;
        bool update_servos_ros = false;
        uint8_t dropped_packets = 0;
        uint8_t ros_buffer[constants::serial::BUFFER_LEN] = {};
        uint8_t radio_buffer[5] = {0};
        uint8_t radio_flag = 1;
    }
}
