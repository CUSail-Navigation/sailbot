#pragma once
#include <Arduino.h>

namespace constants {
    namespace anemometer {
        constexpr uint8_t ANEMOMETER_PIN = 18;
    }
    namespace servo {
        constexpr uint8_t SAIL_PIN = 3;
        constexpr uint8_t RUDDER_PIN = 4;

        constexpr uint8_t SAIL_MIN_ANGLE = 0;
        constexpr uint8_t SAIL_MAX_ANGLE = 90;
        constexpr uint32_t SAIL_MIN_PULSE = 600;    // 1ms pulse width.
        constexpr uint32_t SAIL_MAX_PULSE = 2400;   // 2ms pulse width.

        constexpr uint8_t RUDDER_MIN_ANGLE = 0;
        constexpr uint8_t RUDDER_MAX_ANGLE = 130;   // (2025-2026) 0-130 range = -65 to 65 degrees.
        constexpr uint32_t RUDDER_MIN_PULSE = 600;    // 1ms pulse width.
        constexpr uint32_t RUDDER_MID_PULSE = 1500;
        constexpr uint32_t RUDDER_MAX_PULSE = 2400;   // 2ms pulse width.
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