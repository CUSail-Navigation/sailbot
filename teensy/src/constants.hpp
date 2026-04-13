#pragma once
#include <Arduino.h>

namespace constants {
    namespace anemometer {
        constexpr uint8_t ANEMOMETER_PIN = 18;
    }
    namespace servo {
        constexpr uint8_t RUDDER_PIN = 4;
        constexpr uint8_t MAINSAIL_PIN = 3;
        constexpr uint8_t JIB_PORT_PIN = 0; //TODO
        constexpr uint8_t JIB_STB_PIN = 0;  //TODO

        // (2025-2026) Rudder servo pulse widths (µs) for attach(), mapping, and writes. This is the range that
        // corresponds to full rudder travel on our hull linkage—not the generic 600-2400 span for the servo class.
        constexpr uint32_t RUDDER_MIN_PULSE = 850;
        constexpr uint32_t RUDDER_MAX_PULSE = 2150;
        constexpr uint32_t RUDDER_MID_PULSE = (RUDDER_MIN_PULSE + RUDDER_MAX_PULSE) / 2; // Amidships.
        constexpr uint8_t RUDDER_MIN_ANGLE = 0;
        constexpr uint8_t RUDDER_MAX_ANGLE = 130;       // (2025-2026) 0-130 range = -65 to +65 degrees.

        constexpr uint32_t MAINSAIL_MIN_PULSE = 600;    // 1ms pulse width.
        constexpr uint32_t MAINSAIL_MAX_PULSE = 2400;   // 2ms pulse width.
        constexpr uint8_t MAINSAIL_MIN_ANGLE = 0;
        constexpr uint8_t MAINSAIL_MAX_ANGLE = 90;

        constexpr uint32_t JIB_PORT_MIN_PULSE = 0;  //TODO
        constexpr uint32_t JIB_PORT_MAX_PULSE = 0;  //TODO
        constexpr uint32_t JIB_STB_MIN_PULSE = 0;   //TODO
        constexpr uint32_t JIB_STB_MAX_PULSE = 0;   //TODO
        constexpr uint8_t JIB_MIN_ANGLE = 0;        //TODO
        constexpr uint8_t JIB_MAX_ANGLE = 0;        //TODO
    }
    namespace serial {
        constexpr uint8_t TX_START_FLAG = 0XFF;
        constexpr uint8_t TX_END_FLAG = 0xEE;
        constexpr uint32_t TX_PERIOD_MS = 500;

        constexpr uint8_t RX_START_FLAG = 0xFF;
        constexpr uint8_t RX_END_FLAG = 0xEE;

        constexpr uint32_t BAUD_RATE = 9600;
    }
    namespace led {
        constexpr uint8_t LED_PIN = 13;
    }
}