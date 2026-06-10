#include "MainControlLoop.hpp"

MainControlLoop mcl;

void setup()
{
  Serial.begin(constants::serial::BAUD_RATE);   // Jetson via USB
  Serial2.begin(constants::serial::BAUD_RATE);  // XBee on pins 7/8
}

void loop() {
    mcl.execute();
}