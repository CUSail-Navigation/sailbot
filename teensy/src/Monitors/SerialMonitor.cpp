#include "SerialMonitor.hpp"

SerialMonitor::SerialMonitor()
    : buffer_index(0),
      packet_started(false)
{
}

void SerialMonitor::execute()
{
    while (Serial.available())
    {
        uint8_t incoming_byte = Serial.read();
        // only set flags if packet start byte is correct
        if (incoming_byte == constants::serial::RX_START_FLAG)
        {
            packet_started = true;
            buffer_index = 0;
        }
        // check if packet end byte is correct and buffer is full
        else if (incoming_byte == constants::serial::RX_END_FLAG && buffer_index == sizeof(sfr::serial::buffer) && packet_started)
        {
            buffer_index = 0;
            packet_started = false;
            sfr::serial::update_servos = true;
            sfr::servo::sail_angle = sfr::serial::buffer[0];
            sfr::servo::rudder_angle = sfr::serial::buffer[1];
            sfr::serial::buoy_displacement = sfr::serial::buffer[2];
        }
        else if (packet_started)
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
