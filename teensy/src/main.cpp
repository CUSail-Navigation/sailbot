#include "MainControlLoop.hpp"

MainControlLoop mcl;

void setup(){
    Serial.begin(constants::serial::BAUD_RATE);  // The Jetson via USB.
    Serial2.begin(constants::serial::BAUD_RATE);            // The XBee on pins 7/8.
}

void loop() {
    mcl.execute();
}