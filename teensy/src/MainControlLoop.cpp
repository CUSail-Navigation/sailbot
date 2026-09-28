#include "MainControlLoop.hpp"

/** Delay startup by 1 second to let hardware/peripherals stabilize before the main loop begins. */
MainControlLoop::MainControlLoop() {
    delay(1000);
}

/**
 * Runs a single iteration of the boat's control loop, in the given order.
 */
void MainControlLoop::execute() {
    anemometer_monitor.execute();
    radio_serial_monitor.execute();
    usb_serial_monitor.execute();
    servo_control_task.execute();
    telemetry_control_task.execute();
}
