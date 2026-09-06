#include "RadioSerialMonitor.hpp"

void RadioSerialMonitor::execute() {
    // Catch and drop stale packets that started being processed in earlier execute() calls but stalled.
    if (packet_timed_out()) drop_packet();

    while (Serial2.available()) {
        // Drop the packet in case of timing out while processing it.
        if (packet_timed_out()) drop_packet();

        const uint8_t incoming_byte = Serial2.read();
        if (incoming_byte == constants::serial::RX_START_FLAG) {
            buffer_index = 0;
            packet_started = true;
            packet_start_time = millis();
        }
        else if (packet_started && incoming_byte != constants::serial::RX_END_FLAG) {
            if (buffer_index < sizeof(sfr::serial::radio_buffer)) sfr::serial::radio_buffer[buffer_index++] = incoming_byte;
            else drop_packet(); // Packet is incorrect (buffer is full, but we have not reached RX_END_FLAG).
        }
        else if (packet_started && incoming_byte == constants::serial::RX_END_FLAG) {
            if (buffer_index == sizeof(sfr::serial::radio_buffer)) {
                buffer_index = 0;
                packet_started = false;
                sfr::serial::update_servos_radio = true;
                sfr::serial::radio_flag = sfr::serial::radio_buffer[0];
            }
            else drop_packet(); // Packet is incorrect (buffer is not full, but we have reached RX_END_FLAG).
        }
    }
}
