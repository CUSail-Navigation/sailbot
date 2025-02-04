#include "MainControlLoop.hpp"

MainControlLoop mcl;

// #include <Arduino.h>
// #include <Servo.h>
// Servo myservo;
void setup()
{
  Serial.begin(constants::serial::BAUD_RATE);
}
void loop()
{
  mcl.execute();
}
