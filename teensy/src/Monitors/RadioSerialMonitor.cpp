#include "RadioSerialMonitor.hpp"

/** Reads and assembles incoming radio packets from \code Serial2\endcode into \code radio_buffer\endcode. */
void RadioSerialMonitor::execute() {
    if (read_packet(Serial2, temp_buffer, sfr::serial::radio_buffer, constants::serial::RADIO_BUFFER_LEN)) {
        sfr::serial::update_servos_radio = true;
        sfr::serial::radio_flag = sfr::serial::radio_buffer[0];
    }
}
