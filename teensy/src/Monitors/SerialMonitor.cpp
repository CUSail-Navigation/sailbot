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
        // Reading beginning of the packet: only set flags if the packet start byte is correct.
        if (new_byte == constants::serial::RX_START_FLAG) {
            buffer_index = 0;
            packet_started = true;
            packet_start_time = millis();
        }
        // Reading from the middle of the packet: store a data byte into the buffer.
        else if (packet_started) {
            if (buffer_index < sfr::serial::buffer_length) sfr::serial::buffer[buffer_index++] = new_byte;
            else drop_packet(); // Packet is incorrect (buffer full; RX_END_FLAG not where it should be).
        }
        // Reading end of the packet: check if the packet end byte is correct and the buffer is full.
        else if (new_byte == constants::serial::RX_END_FLAG && buffer_index == sfr::serial::buffer_length && packet_started) {
            buffer_index = 0;
            packet_started = false;
            sfr::serial::update_servos = true;
        }
    }
}

/** Helper method used to indicate dropping a stale packet. */
void SerialMonitor::drop_packet() {
    buffer_index = 0;
    packet_started = false;
    sfr::serial::dropped_packets++;
}