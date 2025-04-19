#pragma once
#include "constants.hpp"
#include "sfr.cpp"
#include "sfr.hpp"

class LEDControl
{
public:
    LEDControl();
    void execute();
private:
    uint8_t ledPin;
};