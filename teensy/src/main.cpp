#include "MainControlLoop.hpp"
#include <Arduino.h>

MainControlLoop mcl; 

void setup()
{
  Serial.begin(constants::serial::BAUD_RATE);
}

void loop()
{
  mcl.execute();
}
