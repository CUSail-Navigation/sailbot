#include "MainControlLoop.hpp"

MainControlLoop mcl;

void setup()
{
  Serial.begin(constants::serial::BAUD_RATE);
  Serial1.begin(constants::serial::BAUD_RATE);
}
void loop()
{
  mcl.execute();
}
