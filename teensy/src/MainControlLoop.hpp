#pragma once

#include "Monitors/AnemometerMonitor.hpp"
#include "Monitors/SerialMonitor.hpp"
#include "Control Tasks/ServoControlTask.hpp"
#include "Control Tasks/SerialControlTask.hpp"
//#include "Control Tasks/TrackerControlTask.hpp"

#include "sfr.hpp"

class MainControlLoop
{
public:
    MainControlLoop();
    void execute();

protected:
    AnemometerMonitor anemometer_monitor;
    SerialMonitor serial_monitor;
    ServoControlTask servo_control_task;
    SerialControlTask serial_control_task;
   // TrackerControlTask tracker_control_task;
};