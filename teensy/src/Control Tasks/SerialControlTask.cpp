#include "SerialControlTask.hpp"

SerialControlTask::SerialControlTask()
    : last_telemetry_send_time(0),
      current_time(0),
      send_telemetry(false)
{
}

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
            (uint8_t)(sfr::anemometer::wind_angle >> 8),
            (uint8_t)(sfr::anemometer::wind_angle & 0xFF),
            sfr::servo::radio_sail_angle,
            sfr::servo::radio_rudder_angle,
            sfr::serial::dropped_packets,
            constants::serial::TX_END_FLAG};

        last_telemetry_send_time = millis();
        Serial2.write(data, sizeof(data));
        send_telemetry = false;
    }
    current_time = millis();
}