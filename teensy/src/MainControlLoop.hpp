#pragma once
#include "Monitors/AnemometerMonitor.hpp"
#include "Monitors/RadioSerialMonitor.hpp"
#include "Monitors/USBSerialMonitor.hpp"
#include "ControlTasks/ServoControlTask.hpp"
#include "ControlTasks/SerialControlTask.hpp"
#include "sfr.hpp"

class MainControlLoop {
public:
    MainControlLoop();
    void execute();

protected:
    AnemometerMonitor anemometer_monitor;
    RadioSerialMonitor radio_serial_monitor;
    USBSerialMonitor usb_serial_monitor;
    ServoControlTask servo_control_task;
    SerialControlTask serial_control_task;
};
