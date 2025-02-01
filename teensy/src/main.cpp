#include "MainControlLoop.hpp"

MainControlLoop mcl;

void setup()
{
  //Serial.print("in setup");
  Serial.begin(constants::serial::BAUD_RATE);
}
void loop()
{
  //Serial.print("in loop");

  mcl.execute();
}
