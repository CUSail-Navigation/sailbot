#pragma once
#include <Arduino.h>

namespace constants {
    namespace anemometer {
        constexpr uint8_t ANEMOMETER_PIN = 18;
    }
    /** SERVO NOTES FROM 2025-2026 SEASON: <p>
     * - All servos can go from 600 PWM (a "minimum angle") to 2400 PWM (a "maximum angle").
     * - Rudder: This servo can turn 0.5 times. Since the rudder goes from -65 degrees to 65 degrees on the hull, the
     *              corresponding PWM range is 850 - 2150.
     * - Mainsail: This servo can turn 7.85 times. PWM of 600 is "all-in", 2400 is "all-out" (on both sides).
     */
    namespace servo {
        constexpr uint32_t SERVO_MIN_PULSE = 600;   // 1ms pulse width. //TODO double check this with Emith.
        constexpr uint32_t SERVO_MAX_PULSE = 2400;  // 2ms pulse width.

        constexpr uint8_t RUDDER_PIN = 4;
        constexpr uint8_t MAINSAIL_PIN = 3;
        constexpr uint8_t JIB_PORT_PIN = 0; //TODO
        constexpr uint8_t JIB_STB_PIN = 0;  //TODO

        constexpr uint32_t RUDDER_MIN_PULSE = 850;
        constexpr uint32_t RUDDER_MAX_PULSE = 2150;
        constexpr uint32_t RUDDER_MID_PULSE = (RUDDER_MIN_PULSE + RUDDER_MAX_PULSE) / 2; // Amidships.
        constexpr uint8_t RUDDER_MIN_ANGLE = 0;
        constexpr uint8_t RUDDER_MAX_ANGLE = 130;       // (2025-2026) 0-130 range = -65 to +65 degrees.

        constexpr uint32_t MAINSAIL_MIN_PULSE = SERVO_MIN_PULSE;
        constexpr uint32_t MAINSAIL_MAX_PULSE = SERVO_MAX_PULSE;
        constexpr uint8_t MAINSAIL_MIN_ANGLE = 0;
        constexpr uint8_t MAINSAIL_MAX_ANGLE = 90;

        constexpr uint32_t JIB_PORT_MIN_PULSE = 0;  //TODO
        constexpr uint32_t JIB_PORT_MAX_PULSE = 0;  //TODO
        constexpr uint32_t JIB_STB_MIN_PULSE = 0;   //TODO
        constexpr uint32_t JIB_STB_MAX_PULSE = 0;   //TODO
        constexpr uint8_t JIB_MIN_ANGLE = 0;        //TODO
        constexpr uint8_t JIB_MAX_ANGLE = 0;        //TODO
    }
    /** SERIAL NOTES FROM 2025-2026 SEASON: <p>
     * - BUFFER FORMAT (which is RX PACKET FORMAT between the start and end flags):
     *   - [0] = \code mainsail_angle\endcode
     *   - [1] = \code rudder_angle\endcode
     *   - [2] = \code jib_port_angle\endcode
     *   - [3] = \code jib_stb_angle\endcode
     * - TX PACKET FORMAT: [start_flag, wind_hi, wind_lo, mainsail_angle, rudder_angle,
     *                      jib_port_angle, jib_stb_angle, dropped_packets, end_flag]
     */
    namespace serial {
        constexpr uint8_t TX_START_FLAG = 0XFF;
        constexpr uint8_t TX_END_FLAG = 0xEE;
        constexpr uint32_t TX_PERIOD_MS = 500;

        constexpr uint8_t RX_START_FLAG = 0xFF;
        constexpr uint8_t RX_END_FLAG = 0xEE;
        constexpr uint32_t RX_PACKET_TIMEOUT_MS = 50; //TODO confirm this number makes sense for a 9600 baud rate

        constexpr uint32_t BAUD_RATE = 9600;
    }
    namespace led {
        constexpr uint8_t LED_PIN = 13;
    }
}