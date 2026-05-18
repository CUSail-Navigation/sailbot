#include "RadioSerialMonitor.hpp"

RadioSerialMonitor::RadioSerialMonitor()
    : buffer_index(0),
      packet_started(false),
      packet_start_time(0)
{
}

void RadioSerialMonitor::execute()
{
    while (Serial2.available())
    {
        uint8_t incoming_byte = Serial2.read();

        // Start of packet
        if (incoming_byte == constants::serial::RX_START_FLAG)
        {
            packet_started = true;
            buffer_index = 0;
            packet_start_time = millis();
        }
        // End of packet and full RADIO payload received
        else if (incoming_byte == constants::serial::RX_END_FLAG &&
                 buffer_index == sizeof(sfr::serial::radio_buffer) &&
                 packet_started)
        {
            packet_started = false;
            buffer_index = 0;

            // Decode RADIO packet: [radio_flag, sail, rudder]
            sfr::serial::update_servos_radio = true;
            sfr::serial::radio_flag        = sfr::serial::radio_buffer[0];
            sfr::servo::radio_sail_angle   = sfr::serial::radio_buffer[1];
            sfr::servo::radio_rudder_angle = sfr::serial::radio_buffer[2];

            Serial.print("Received RADIO packet: flag=");
            Serial.println(sfr::serial::radio_flag);
            Serial.print(", sail=");
            Serial.println(sfr::servo::radio_sail_angle);
            Serial.print(", rudder=");
            Serial.println(sfr::servo::radio_rudder_angle);
        }
        
        else if (packet_started)
        {
            // Store data byte into local radio buffer
            if (buffer_index < sizeof(sfr::serial::radio_buffer))
            {
                sfr::serial::radio_buffer[buffer_index++] = incoming_byte;
            }
            else
            {
                // Buffer overflow → drop packet
                buffer_index = 0;
                packet_started = false;
                sfr::serial::dropped_packets++;
            }
        }
    }
}
