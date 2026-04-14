#include "SerialMonitor.hpp"

SerialMonitor::SerialMonitor()
    : buffer_index(0),
      packet_started(false)
{}

void SerialMonitor::execute() {
    // RX packet format (between start/end flags):
    // [0]=mainsail_angle, [1]=rudder_angle, [2]=jib_port_angle, [3]=jib_stb_angle
    while (Serial.available()) {
        uint8_t incoming_byte = Serial.read();
        // Only set flags if packet start byte is correct.
        if (incoming_byte == constants::serial::RX_START_FLAG) {
            packet_started = true;
            buffer_index = 0;
        }
        // Check if packet end byte is correct and buffer is full.
        else if (incoming_byte == constants::serial::RX_END_FLAG &&
            buffer_index == sizeof(sfr::serial::buffer) && packet_started) {
            buffer_index = 0;
            packet_started = false;
            sfr::serial::update_servos = true;
            sfr::servo::mainsail_angle = sfr::serial::buffer[0];
            sfr::servo::rudder_angle = sfr::serial::buffer[1];
            sfr::servo::jib_port_angle = sfr::serial::buffer[2];
            sfr::servo::jib_stb_angle = sfr::serial::buffer[3];
        }
        else if (packet_started) {
            // Store a data byte into the buffer.
            if (buffer_index < sizeof(sfr::serial::buffer)) sfr::serial::buffer[buffer_index++] = incoming_byte;
            // If buffer is full, drop the packet.
            else {
                buffer_index = 0;
                packet_started = false;
                sfr::serial::dropped_packets++;
            }
        }
    }
}