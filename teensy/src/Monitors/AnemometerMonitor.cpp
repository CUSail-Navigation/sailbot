#include "AnemometerMonitor.hpp"

AnemometerMonitor::AnemometerMonitor()
{
    pinMode(constants::anemometer::ANEMOMETER_PIN, INPUT);
}

void AnemometerMonitor::execute()
{  
    sfr::anemometer::wind_angle = ((0.97826))*((360*analogRead(constants::anemometer::ANEMOMETER_PIN))/1000);//0.97826 = 360/368

    Serial.println(analogRead(constants::anemometer::ANEMOMETER_PIN)); //print for testing
    //adjusted value converts 368 to 360
}