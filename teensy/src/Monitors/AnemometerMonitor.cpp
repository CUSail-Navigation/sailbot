#include "AnemometerMonitor.hpp"

AnemometerMonitor::AnemometerMonitor() {
    pinMode(constants::anemometer::ANEMOMETER_PIN, INPUT);
}

void AnemometerMonitor::execute() {
    // Note that 0.97826 = 360/368 -- adjusted value converts 368 to 360.
    sfr::anemometer::wind_angle = (0.97826)*((360*analogRead(constants::anemometer::ANEMOMETER_PIN))/1000);
    //Serial.println(sfr::anemometer::wind_angle); // Print for testing.
}