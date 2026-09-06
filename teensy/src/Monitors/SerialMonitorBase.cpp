#include "SerialMonitorBase.hpp"

SerialMonitorBase::SerialMonitorBase() : buffer_index(0), packet_started(false), packet_start_time(0) {}

/** Helper method used to indicate dropping a stale packet. */
void SerialMonitorBase::drop_packet() {
    buffer_index = 0;
    packet_started = false;
    sfr::serial::dropped_packets++;
}

/** Returns true when an in-progress packet has exceeded the RX timeout. */
bool SerialMonitorBase::packet_timed_out() const {
    return packet_started && (millis() - packet_start_time > constants::serial::RX_PACKET_TIMEOUT_MS);
}
