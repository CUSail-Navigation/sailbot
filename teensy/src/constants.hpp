#pragma once
#include <Arduino.h>

namespace constants {
    namespace anemometer {
        constexpr uint8_t ANEMOMETER_PIN = 18; //TODO
    }
    /** SERVO NOTES FROM 2025-2026 SEASON: <p>
     * - All servos can go from 600 PWM (a "minimum angle") to 2400 PWM (a "maximum angle").
     * - Rudder: This servo can turn 0.5 times, but mech did something with the gears so that the rudder will only go
     *              from -45 degrees (600 PWM) to 45 degrees (2400 PWM).
     * - Mainsail: This servo can turn 7.85 times, but [TODO]. PWM of [TODO] is "all-in", [TODO] is "all-out" (on both sides).
     * - Jib: [TODO].
     */
    namespace servo {
        constexpr uint32_t SERVO_MIN_PULSE = 600;
        constexpr uint32_t SERVO_MAX_PULSE = 2400;

        constexpr uint8_t RUDDER_PIN = 4;   //TODO
        constexpr uint8_t MAINSAIL_PIN = 3; //TODO
        constexpr uint8_t JIB_PORT_PIN = 0; //TODO
        constexpr uint8_t JIB_STB_PIN = 0;  //TODO

        constexpr uint32_t RUDDER_MIN_PULSE = SERVO_MIN_PULSE;
        constexpr uint32_t RUDDER_MAX_PULSE = SERVO_MAX_PULSE;
        constexpr uint32_t RUDDER_MID_PULSE = (RUDDER_MIN_PULSE + RUDDER_MAX_PULSE) / 2; // Amidships.
        constexpr uint8_t RUDDER_MIN_ANGLE = 0;
        constexpr uint8_t RUDDER_MAX_ANGLE = 90; // (2025-2026) 0-90 range = -45 to +45 degrees.

        constexpr uint32_t MAINSAIL_MIN_PULSE = 0;  //TODO
        constexpr uint32_t MAINSAIL_MAX_PULSE = 0;  //TODO
        constexpr uint8_t MAINSAIL_MIN_ANGLE = 0;
        constexpr uint8_t MAINSAIL_MAX_ANGLE = 90;
        constexpr uint8_t BOOM_LENGTH_CM_SQUARED = 0 * 0; //TODO

        constexpr uint32_t JIB_PORT_MIN_PULSE = 0;  //TODO
        constexpr uint32_t JIB_PORT_MAX_PULSE = 0;  //TODO
        constexpr uint32_t JIB_STB_MIN_PULSE = 0;   //TODO
        constexpr uint32_t JIB_STB_MAX_PULSE = 0;   //TODO
        constexpr uint8_t JIB_MIN_ANGLE = 15;
        constexpr uint8_t JIB_MAX_ANGLE = 75;
        constexpr uint8_t JIB_FOOT_LENGTH_CM_SQUARED = 0 * 0; //TODO

        /** Active jib sheet side (RX/TX and \code sfr::servo::jib_side_flag\endcode). */
        constexpr uint8_t JIB_SIDE_PORT = 0;
        constexpr uint8_t JIB_SIDE_STARBOARD = 1;
    }
    /** SERIAL NOTES FROM 2025-2026 SEASON: <p>
     * - BUFFER FORMAT (which is RX PACKET FORMAT between the start and end flags):
     *   - [0] = \code mainsail_angle\endcode
     *   - [1] = \code rudder_angle\endcode
     *   - [2] = \code jib_angle\endcode
     *   - [3] = \code jib_side_flag\endcode (\code JIB_SIDE_PORT\endcode / \code JIB_SIDE_STARBOARD\endcode)
     * - TX PACKET FORMAT: [start_flag, wind_hi, wind_lo, mainsail_angle, rudder_angle,
     *                      jib_angle, jib_side_flag, dropped_packets, end_flag]
     */
    namespace serial {
        constexpr uint8_t TX_START_FLAG = 0XFF;
        constexpr uint8_t TX_END_FLAG = 0xEE;
        constexpr uint32_t TX_PERIOD_MS = 500;

        constexpr uint8_t RX_START_FLAG = 0xFF;
        constexpr uint8_t RX_END_FLAG = 0xEE;
        constexpr uint8_t RX_PACKET_TIMEOUT_MS = 50;
        constexpr uint8_t BUFFER_LEN = 4;

        constexpr uint32_t BAUD_RATE = 9600;
    }
    namespace led {
        constexpr uint8_t LED_PIN = 13; // TODO
    }
}