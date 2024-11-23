#include "MainControlLoop.hpp"

MainControlLoop::MainControlLoop()
    : anemometer_monitor(),
      serial_monitor(),
      servo_control_task(),
      serial_control_task(),
      tracker_control_task()
{
    delay(1000);
}

void MainControlLoop::execute()
{
    anemometer_monitor.execute();
    serial_monitor.execute();
    servo_control_task.execute();
    serial_control_task.execute();
    tracker_control_task.execute();
}
