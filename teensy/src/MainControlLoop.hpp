#pragma once
#include "Monitors/AnemometerMonitor.hpp"
#include "Monitors/RadioSerialMonitor.hpp"
#include "Monitors/ROSSerialMonitor.hpp"
#include "Control Tasks/ServoControlTask.hpp"
#include "Control Tasks/SerialControlTask.hpp"
#include "sfr.hpp"

class MainControlLoop {
public:
    MainControlLoop();
    void execute();

protected:
    AnemometerMonitor anemometer_monitor;
    RadioSerialMonitor radio_serial_monitor;
    ROSSerialMonitor ros_serial_monitor;
    ServoControlTask servo_control_task;
    SerialControlTask serial_control_task;
};