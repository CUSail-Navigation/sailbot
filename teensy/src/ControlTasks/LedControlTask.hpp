#pragma once
#include "sfr.hpp"

class LedControlTask {
public:
    LedControlTask();
    void execute() const;

private:
    const uint8_t LED_PIN;
};
