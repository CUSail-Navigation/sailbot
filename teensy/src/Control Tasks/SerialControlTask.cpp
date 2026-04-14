#include "SerialControlTask.hpp"

SerialControlTask::SerialControlTask()
    : last_telemetry_send_time(0),
      current_time(0),
      send_telemetry(false)
{}

void SerialControlTask::execute() {
    if (last_telemetry_send_time - current_time >= constants::serial::TX_PERIOD_MS) {
        send_telemetry = true;
    }
    if (send_telemetry) {
        // TX packet format:
        // [start_flag, wind_hi, wind_lo, mainsail_angle, rudder_angle, jib_port_angle, jib_stb_angle, dropped_packets, end_flag]
        uint8_t data[] = {
            constants::serial::TX_START_FLAG,
            sfr::anemometer::wind_angle >> 8,
            sfr::anemometer::wind_angle & 0xFF,
            sfr::servo::mainsail_angle,
            sfr::servo::rudder_angle,
            sfr::servo::jib_port_angle,
            sfr::servo::jib_stb_angle,
            sfr::serial::dropped_packets,
            constants::serial::TX_END_FLAG};

        last_telemetry_send_time = millis();
        Serial.write(data, sizeof(data));
        send_telemetry = false;
    }
    current_time = millis();
}