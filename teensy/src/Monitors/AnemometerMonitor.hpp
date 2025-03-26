#pragma once

#include <Arduino.h>
#include "sfr.hpp"
#include "constants.hpp"

class AnemometerMonitor
{
public:
    AnemometerMonitor();
    void execute();
};