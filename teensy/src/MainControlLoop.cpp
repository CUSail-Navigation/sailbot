#include "MainControlLoop.hpp"

MainControlLoop::MainControlLoop() {
    delay(1000);
}

void MainControlLoop::execute() {
    anemometer_monitor.execute();
    radio_serial_monitor.execute();
    ros_serial_monitor.execute();
    servo_control_task.execute();
    serial_control_task.execute();
}
