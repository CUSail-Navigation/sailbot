#include "SerialMonitor.hpp"

SerialMonitor::SerialMonitor()
    : buffer_index(0),
      packet_started(false),
      packet_start_time(0)
{
}

void SerialMonitor::execute()
{
    while (Serial.available())
    {
        if (millis() - packet_start_time >= constants::serial::RX_TIMEOUT_MS)
        {
            buffer_index = 0;
            packet_started = false;
            sfr::serial::dropped_packets++;
            return;
        }

        uint8_t incoming_byte = Serial.read();
        // no part of the packet has been received
        if (!packet_started)
        {
            // only set flags if packet start byte is correct
            if (incoming_byte == constants::serial::RX_START_FLAG)
            {
                packet_start_time = millis();
                packet_started = true;
            }
            sfr::serial::dropped_packets++;
        }
        else
        {
            // we receive a packet delimter byte, this means a complete packet was received
            if (
                incoming_byte == constants::serial::RX_END_FLAG &&
                buffer_index == sizeof(sfr::serial::buffer) + 1) // +1 for the packet end byte
            {
                buffer_index = 0;
                packet_started = false;
                sfr::serial::update_servos = true;
            }
            else
            {
                // store a data byte into the buffer
                if (buffer_index < sizeof(sfr::serial::buffer))
                    sfr::serial::buffer[buffer_index++] = incoming_byte;
                // if buffer is full, drop the packet
                else
                {
                    buffer_index = 0;
                    packet_started = false;
                    sfr::serial::dropped_packets++;
                }
            }
        }
    }
}
