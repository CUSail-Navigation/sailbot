#include "SerialMonitor.hpp"

SerialMonitor::SerialMonitor()
    : buffer_index(0),
      packet_started(false)
{
}

void SerialMonitor::execute()
{
    //Serial.println("is executing");
    while (Serial.available())
    {
        uint8_t incoming_byte = Serial.read();
        Serial.println(incoming_byte);
        Serial.println("buffer conditionals");
        Serial.println(buffer_index);
        Serial.println(sizeof(sfr::serial::buffer));
        Serial.println(packet_started);
        Serial.println(incoming_byte == constants::serial::RX_END_FLAG);
        // only set flags if packet start byte is correct
        if (incoming_byte == constants::serial::RX_START_FLAG)
        {
            Serial.println("reached serial monitor task 17");
            packet_started = true;
            buffer_index = 0;
        }
        // check if packet end byte is correct and buffer is full


        else if (incoming_byte == constants::serial::RX_END_FLAG && buffer_index == sizeof(sfr::serial::buffer) && packet_started)
        {
            Serial.println("serial monitor 24");
            buffer_index = 0;
            packet_started = false;
            sfr::serial::update_servos = true;
            sfr::servo::sail_angle = sfr::serial::buffer[0];
            sfr::servo::rudder_angle = sfr::serial::buffer[1];
            sfr::serial::servo_angle = sfr::serial::buffer[2];
            Serial.println(sfr::serial::update_servos);
        }
        else if (packet_started)
        {
            Serial.println("packet started");
            // store a data byte into the buffer
            if (buffer_index < sizeof(sfr::serial::buffer)) {
                sfr::serial::buffer[buffer_index++] = incoming_byte;
                Serial.println("added to buffer");
        }
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
