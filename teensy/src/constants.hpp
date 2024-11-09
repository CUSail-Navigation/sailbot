#include <Arduino.h>

uint8_t test = 0;
namespace constants
{
    namespace anemometer
    {
        constexpr uint8_t ANEMOMETER_PIN = 0; // FIXME: change to correct pin number
    }
    namespace servo
    {
        constexpr uint8_t SAIL_PIN = 4;
        constexpr uint8_t RUDDER_PIN = 3;

        constexpr uint8_t SAIL_MIN_ANGLE = 0;
        constexpr uint8_t SAIL_MAX_ANGLE = 90;

        constexpr uint8_t RUDDER_MIN_ANGLE = 0;
        constexpr uint8_t RUDDER_MAX_ANGLE = 90;
    }
    namespace serial
    {
        constexpr uint8_t TX_START_FLAG = 0XFF;
        constexpr uint8_t TX_END_FLAG = 0xEE;
        constexpr uint32_t TX_PERIOD_MS = 1000;

        constexpr uint8_t RX_START_FLAG = 0xFF;
        constexpr uint8_t RX_END_FLAG = 0xEE;

        constexpr uint8_t BAUD_RATE = 9600;
    }
}