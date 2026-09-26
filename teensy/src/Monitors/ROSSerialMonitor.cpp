#include "ROSSerialMonitor.hpp"

/** Reads and assembles incoming Jetson packets from \code Serial\endcode into \code ros_buffer\endcode. */
void ROSSerialMonitor::execute() {
    if (read_packet(Serial, temp_buffer, sfr::serial::ros_buffer, constants::serial::BUFFER_LEN)) {
        sfr::serial::update_servos_ros = true;
    }
}
