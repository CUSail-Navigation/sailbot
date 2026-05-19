#include "RadioSerialMonitor.hpp"

RadioSerialMonitor::RadioSerialMonitor()
    : buffer_index(0),
      packet_started(false),
      packet_start_time(0)
{
}

void RadioSerialMonitor::execute()
{
    // Drop stale in-progress packets.
    if (packet_started && (millis() - packet_start_time > constants::serial::RX_PACKET_TIMEOUT_MS)) {
        buffer_index = 0;
        packet_started = false;
        sfr::serial::dropped_packets++;
    }

    while (Serial2.available())
    {
        // Also check timeout mid-stream.
        if (packet_started && (millis() - packet_start_time > constants::serial::RX_PACKET_TIMEOUT_MS)) {
            buffer_index = 0;
            packet_started = false;
            sfr::serial::dropped_packets++;
        }

        uint8_t incoming_byte = Serial2.read();

        // Start of packet
        if (incoming_byte == constants::serial::RX_START_FLAG)
        {
            packet_started = true;
            buffer_index = 0;
            packet_start_time = millis();
        }
        // End of packet — accept only when all 5 payload bytes have arrived.
        else if (incoming_byte == constants::serial::RX_END_FLAG &&
                 buffer_index == sizeof(sfr::serial::radio_buffer) &&
                 packet_started)
        {
            packet_started = false;
            buffer_index = 0;

            // Decode RADIO packet: [radio_flag, mainsail_angle, rudder_angle, jib_angle, jib_side_flag]
            sfr::serial::radio_flag          = sfr::serial::radio_buffer[0];
            sfr::servo::radio_sail_angle     = sfr::serial::radio_buffer[1];
            sfr::servo::radio_rudder_angle   = sfr::serial::radio_buffer[2];
            sfr::serial::update_servos_radio = true;
        }
        else if (packet_started)
        {
            if (buffer_index < sizeof(sfr::serial::radio_buffer))
            {
                sfr::serial::radio_buffer[buffer_index++] = incoming_byte;
            }
            else
            {
                // Buffer overflow → drop packet.
                buffer_index = 0;
                packet_started = false;
                sfr::serial::dropped_packets++;
            }
        }
    }
}
