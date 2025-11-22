#include "MainControlLoop.hpp"

MainControlLoop::MainControlLoop()
    : anemometer_monitor(),
      ros_serial_monitor(),
      radio_serial_monitor(),
      servo_control_task(),
      serial_control_task()
{

    delay(1000);
}

void MainControlLoop::execute()
{
    anemometer_monitor.execute();
    ros_serial_monitor.execute();
    radio_serial_monitor.execute();
    servo_control_task.execute();
    serial_control_task.execute();
}
