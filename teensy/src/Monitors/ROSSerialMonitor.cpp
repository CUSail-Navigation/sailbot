#include "ROSSerialMonitor.hpp"

ROSSerialMonitor::ROSSerialMonitor()
    : buffer_index(0),
      packet_started(false),
      packet_start_time(0)
{
}

void ROSSerialMonitor::execute()
{
    while (Serial.available())
    {
        uint8_t incoming_byte = Serial.read();

        // Start of packet
        if (incoming_byte == constants::serial::RX_START_FLAG)
        {
            packet_started = true;
            buffer_index = 0;
            packet_start_time = millis();
        }
        // End of packet and full ROS payload received
        else if (incoming_byte == constants::serial::RX_END_FLAG &&
                 buffer_index == sizeof(sfr::serial::ros_buffer) &&
                 packet_started)
        {
            packet_started = false;
            buffer_index = 0;

            // Decode ROS packet: [sail, rudder]
            sfr::serial::update_servos_ros = true;
            // sfr::servo::ros_sail_angle   = sfr::serial::ros_buffer[0]; //todo delete
            // sfr::servo::ros_rudder_angle = sfr::serial::ros_buffer[1]; //todo delete
        }
        else if (packet_started)
        {
            // Store data byte into the buffer
            if (buffer_index < sizeof(sfr::serial::ros_buffer))
            {
                sfr::serial::ros_buffer[buffer_index++] = incoming_byte;
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
