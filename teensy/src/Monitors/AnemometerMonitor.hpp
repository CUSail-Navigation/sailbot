#pragma once

#include <Arduino.h>
#include "sfr.hpp"
#include "constants.hpp"

class AnemometerMonitor
{
private:
    float wind_speed;
    volatile unsigned long elapsed_time;
    volatile unsigned long last_pulse_ms_isr;
    volatile bool have_period;
    unsigned long last_pulse_time;
    unsigned long period_ms;
    unsigned long last_ms;

    bool ok;

    static void windSpeedISR();

    static AnemometerMonitor* _instance;

public:
    AnemometerMonitor();
    void execute();
};
