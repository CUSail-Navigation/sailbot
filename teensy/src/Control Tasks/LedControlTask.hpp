#pragma once
#include "sfr.hpp"
#include <Arduino.h>

class LedControlTask {
public:
    LedControlTask();
    void execute();

private:
    int LED_PIN = constants::led::LED_PIN;
};