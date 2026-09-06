#include "AnemometerMonitor.hpp"

AnemometerMonitor::AnemometerMonitor() {
    pinMode(constants::anemometer::ANEMOMETER_PIN, INPUT);
}

void AnemometerMonitor::execute() {
    // Note that 0.3515625 = 360/1024 -- maps the analogRead() range of 0-1023 onto 0-359 degrees.
    sfr::anemometer::wind_angle = static_cast<int>(0.3515625 * analogRead(constants::anemometer::ANEMOMETER_PIN));
    //Serial.println(sfr::anemometer::wind_angle); // Print for testing.
}
