#include "SerialControlTask.hpp"

SerialControlTask::SerialControlTask() : last_telemetry_send_time(0), current_time(0), send_telemetry(false) {}

void SerialControlTask::execute()
{
    if (current_time - last_telemetry_send_time >= constants::serial::TX_PERIOD_MS)
    {
        send_telemetry = true;
    }
    if (send_telemetry)
    {
        uint8_t data[] = {
            constants::serial::TX_START_FLAG,
            static_cast<uint8_t>((uint8_t)(sfr::anemometer::wind_angle >> 8)),
            static_cast<uint8_t>((uint8_t)(sfr::anemometer::wind_angle & 0xFF)),
            sfr::servo::mainsail_angle,
            sfr::servo::rudder_angle,
            sfr::servo::jib_angle,
            sfr::servo::jib_side_flag,
            sfr::serial::dropped_packets,
            constants::serial::TX_END_FLAG
        };

        last_telemetry_send_time = millis();
        Serial.write(data, sizeof(data));
        send_telemetry = false;
    }
    current_time = millis();
}