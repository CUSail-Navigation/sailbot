#include "AnemometerMonitor.hpp"

AnemometerMonitor::AnemometerMonitor() {
    pinMode(constants::anemometer::ANEMOMETER_PIN, INPUT);
}

void AnemometerMonitor::execute() {
    // Note that 0.35191 = 360/1023 -- maps analogRead() range of 0-1023 to 0-360 degrees.
    sfr::anemometer::wind_angle = static_cast<int>(0.35191 * analogRead(constants::anemometer::ANEMOMETER_PIN));
    //Serial.println(sfr::anemometer::wind_angle); // Print for testing.
}
