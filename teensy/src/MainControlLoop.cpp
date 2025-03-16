#include "MainControlLoop.hpp"

MainControlLoop::MainControlLoop()
   : 
    led_control_task(),
    serial_monitor(),
    //servo_control_task(),
    serial_control_task()
{
    delay(1000);
}

void MainControlLoop::execute()
{  
    led_control_task.execute();
    // anemometer_monitor.execute();
    //serial_monitor.execute();
    //servo_control_task.execute();
    serial_control_task.execute();
}
