#include "SerialMonitor.hpp"

SerialMonitor::SerialMonitor() : buffer_index(0), packet_started(false), packet_start_time(0) {}

/** Process incoming packets and update SFR fields if applicable. */
void SerialMonitor::execute() {
    // Catch and drop stale partial packets that started in earlier execute() calls but stalled.
    if (packet_started && (millis() - packet_start_time > constants::serial::RX_PACKET_TIMEOUT_MS)) drop_packet();

    while (Serial.available()) {
        // In case of timeout during processing, drop the stale partial packet.
        if (packet_started && (millis() - packet_start_time > constants::serial::RX_PACKET_TIMEOUT_MS)) drop_packet();

        const uint8_t new_byte = Serial.read();
        if (new_byte == constants::serial::RX_START_FLAG) {
            buffer_index = 0;
            packet_started = true;
            packet_start_time = millis();
        }
        else if (packet_started && new_byte != constants::serial::RX_END_FLAG) {
            if (buffer_index < sfr::serial::buffer_length) sfr::serial::buffer[buffer_index++] = new_byte;
            else drop_packet(); // Packet is incorrect (buffer is full and RX_END_FLAG is not where it should be).
        }
        else if (packet_started && new_byte == constants::serial::RX_END_FLAG) {
            if (buffer_index == sfr::serial::buffer_length) {
                buffer_index = 0;
                packet_started = false;
                sfr::serial::update_servos = true;
            }
            else drop_packet(); // Packet is incorrect (buffer is not full, but we have reached RX_END_FLAG).
        }
    }
}

/** Helper method used to indicate dropping a stale packet. */
void SerialMonitor::drop_packet() {
    buffer_index = 0;
    packet_started = false;
    sfr::serial::dropped_packets++;
}