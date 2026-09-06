#pragma once
#include "sfr.hpp"

class LedControlTask {
public:
    LedControlTask();
    void execute() const;

private:
    int LED_PIN = constants::led::LED_PIN;
};
