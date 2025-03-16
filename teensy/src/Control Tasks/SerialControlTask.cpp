#include "SerialControlTask.hpp"

SerialControlTask::SerialControlTask()
    : last_telemetry_send_time(0),
      current_time(0),
      send_telemetry(false)
{
}

void SerialControlTask::execute()
{
    if (last_telemetry_send_time - current_time >= constants::serial::TX_PERIOD_MS)
    {
        send_telemetry = true;
    }
    if (send_telemetry)
    {
        uint8_t data[] = {
            constants::serial::TX_START_FLAG,
            sfr::servo::sail_angle,
            sfr::servo::rudder_angle,
            sfr::serial::dropped_packets,
            constants::serial::TX_END_FLAG};

        last_telemetry_send_time = millis();
        Serial.write(data, sizeof(data));
        send_telemetry = false;
    }
    current_time = millis();

}