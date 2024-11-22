#include "sfr.hpp"
#include "constants.hpp"

class SerialControlTask
{
public:
    SerialControlTask();
    void execute();

private:
    uint32_t last_telemetry_send_time;
    uint32_t current_time;
    bool send_telemetry;
};