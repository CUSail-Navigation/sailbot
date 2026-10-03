#pragma once
#include "sfr.hpp"
#include <algorithm>

/** An abstract base class: stores shared functionality for serial monitors that assemble packets. */
class SerialMonitorBase {
public:
    virtual void execute() = 0;

protected:
    uint8_t buffer_index;
    bool packet_started;
    uint32_t packet_start_time;

    SerialMonitorBase();
    virtual ~SerialMonitorBase() = default;

    void drop_packet();
    [[nodiscard]] bool packet_timed_out() const;

    /**
    * Reads and assembles incoming serial packets from \code port\endcode into \code sfr_buffer\endcode, byte by byte.
    * Returns \code true\endcode if at least one full packet is assembled during this call; the last of such packets is
    * reflected in \code sfr_buffer\endcode.
    *
    * Valid packets must begin with \code RX_START_FLAG\endcode, end with \code RX_END_FLAG\endcode, and contain exactly
    * \code len\endcode bytes. Malformed packets that do not follow this structure, or packets that stall for longer
    * than \code RX_PACKET_TIMEOUT_MS\endcode, are dropped.
    */
    template <typename Port> bool read_packet(Port& port, uint8_t* temp_buffer, uint8_t* sfr_buffer, const uint8_t len) {
        if (packet_timed_out()) drop_packet();

        bool completed = false;
        while (port.available()) {
            if (packet_timed_out()) drop_packet();

            if (const uint8_t incoming_byte = port.read(); incoming_byte == constants::serial::RX_START_FLAG) {
                buffer_index = 0;
                packet_started = true;
                packet_start_time = millis();
            }
            else if (packet_started && incoming_byte != constants::serial::RX_END_FLAG) {
                if (buffer_index < len) temp_buffer[buffer_index++] = incoming_byte;
                else drop_packet(); // Packet is incorrect (buffer is full, but we have not reached RX_END_FLAG).
            }
            else if (packet_started && incoming_byte == constants::serial::RX_END_FLAG) {
                if (buffer_index == len) {
                    std::copy_n(temp_buffer, len, sfr_buffer);
                    buffer_index = 0;
                    packet_started = false;
                    completed = true;
                }
                else drop_packet(); // Packet is incorrect (buffer is not full, but we have reached RX_END_FLAG).
            }
        }

        return completed;
    }
};
