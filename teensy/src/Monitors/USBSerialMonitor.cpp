#include "USBSerialMonitor.hpp"

/**
 * Reads and assembles incoming packets from the computer connected via \code Serial\endcode into
 * \code usb_buffer\endcode.
 */
void USBSerialMonitor::execute() {
    if (read_packet(Serial, temp_buffer, sfr::serial::usb_buffer, constants::serial::USB_BUFFER_LEN)) {
        sfr::serial::update_servos_usb = true;
    }
}
