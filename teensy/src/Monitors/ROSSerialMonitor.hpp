#pragma once
#include "SerialMonitorBase.hpp"

class ROSSerialMonitor : public SerialMonitorBase {
public:
    void execute() override;
};
