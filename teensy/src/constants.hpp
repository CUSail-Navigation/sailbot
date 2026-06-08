#pragma once
#include <Arduino.h>

namespace constants {
    namespace anemometer {
        constexpr uint8_t ANEMOMETER_PIN = 18;
    }
    /** PHYSICAL SERVO NOTES FROM 2025-2026 SEASON:
     * - All servos can go from 600 PWM (a "minimum angle") to 2400 PWM (a "maximum angle").
     * - Rudder: This servo can turn 0.5 times, but mech did something to cut this in half, so that the rudder will only
     *              go from -45 degrees (600 PWM) to 45 degrees (2400 PWM).
     * - Mainsail: This servo can turn 7.85 times.
     * - Jib: We have two servos; one for each side of the boat. Both servos can turn 7.85 times.
     *
     * NOTE: Anything labeled TODO is ones that will be made runtime-changable over the mobile app. This
     *              functionality is not implemented yet
     */
    namespace servo {

        constexpr uint8_t RUDDER_PIN   = 4;
        constexpr uint8_t MAINSAIL_PIN = 3;
        constexpr uint8_t JIB_PORT_PIN = 5; 
        constexpr uint8_t JIB_STB_PIN  = 9;

        constexpr uint32_t SERVO_MIN_PULSE = 600;
        constexpr uint32_t SERVO_MAX_PULSE = 2400;

        constexpr uint32_t RUDDER_MIN_PULSE = 600;                                                          //TODO
        constexpr uint32_t RUDDER_MAX_PULSE = 2400;                                                         //TODO

        constexpr uint32_t JIB_PORT_MIN_PULSE = 600;                                                        //TODO
        constexpr uint32_t JIB_PORT_MAX_PULSE = 1700;                                                       //TODO

        constexpr uint32_t JIB_STB_MIN_PULSE = 600;                                                         //TODO
        constexpr uint32_t JIB_STB_MAX_PULSE = 1600;                                                        //TODO

        constexpr uint32_t MAINSAIL_MIN_PULSE = 650;
        constexpr uint32_t MAINSAIL_MAX_PULSE = 2100;

        constexpr uint8_t RUDDER_MIN_ANGLE = 0; // (2025-2026) 0-90 range = -45 to +45 degrees.                         //TODO
        constexpr uint8_t RUDDER_MAX_ANGLE = 90;                                                                        //TODO
        constexpr uint32_t RUDDER_MID_PULSE = (RUDDER_MIN_PULSE + RUDDER_MAX_PULSE) / 2; // Amidships.

        constexpr uint8_t MAINSAIL_MIN_ANGLE = 0;                                                                       //TODO
        constexpr uint8_t MAINSAIL_MAX_ANGLE = 90;                                                                      //TODO
        constexpr uint32_t TWO_BOOM_LEN_SQD_CM = 2 * 92 * 92; // (2025-2026) Boom length (mast to mainsheet) is 92cm.   //TODO
        constexpr uint32_t MAINSAIL_INITIAL_CM = 18; // (2025-2026) Mainsheet length from deck to end of boom is 18cm.  //TODO
        constexpr uint32_t MAINSAIL_INITIAL_SQD_CM = MAINSAIL_INITIAL_CM * MAINSAIL_INITIAL_CM;
        constexpr float MAINSAIL_WHEEL_CIRCUM_CM = 16.242; // (2025-2026) Diameter: 5.17cm.                             //TODO
        constexpr float MAINSAIL_PULSE_PER_TURN = (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 7.85;

        constexpr uint8_t JIB_MIN_ANGLE = 0;                                                                           //TODO
        constexpr uint8_t JIB_MAX_ANGLE = 90;                                                                           //TODO
        constexpr uint32_t TWO_JIB_FOOT_LEN_SQD_CM = 2 * 60 * 60; // (2025-2026) Jib foot length is 60cm.               //TODO
        constexpr float JIB_PORT_WHEEL_CIRCUM_CM = 16.242; // (2025-2026) Diameter: 5.17cm.                             //TODO
        constexpr float JIB_STB_WHEEL_CIRCUM_CM = 16.242; // (2025-2026) Diameter: 5.17cm.                              //TODO
        constexpr float JIB_PULSE_PER_TURN = (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 7.85;

        /** Active jib sheet side (RX/TX and \code sfr::servo::jib_side_flag\endcode). */
        constexpr uint8_t JIB_SIDE_PORT = 0;
        constexpr uint8_t JIB_SIDE_STB = 1;
    }
    /** SERIAL NOTES FROM 2025-2026 SEASON: <p>
     * - BUFFER FORMAT (which is RX PACKET FORMAT between the start and end flags):
     *   - [0] = \code mainsail_angle\endcode
     *   - [1] = \code rudder_angle\endcode
     *   - [2] = \code jib_angle\endcode
     *   - [3] = \code jib_side_flag\endcode ( \code JIB_SIDE_PORT\endcode or \code JIB_SIDE_STARBOARD\endcode )
     * - TX PACKET FORMAT: [start_flag, wind_hi, wind_lo, mainsail_angle, rudder_angle,
     *                      jib_angle, jib_side_flag, dropped_packets, end_flag].
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
        constexpr uint8_t LED_PIN = 13;
    }
}