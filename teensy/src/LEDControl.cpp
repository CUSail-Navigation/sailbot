#include "LEDControl.hpp"

LEDControl::LEDControl()
{
    ledPin = constants::LED::LED_PIN;
    pinMode(ledPin,OUTPUT);
}

void LEDControl::execute()
{
    if(!sfr::serial::update_servos && Serial.available())
    {
        digitalWrite(ledPin, LOW);
        delay(2000);
        digitalWrite(ledPin, HIGH);
        delay(2000);
    }
    else if(sfr::serial::update_servos)
    {
        digitalWrite(ledPin, HIGH);
        delay(1000);
    }
    else if(!Serial.available())
    {
        digitalWrite(ledPin, LOW);
        delay(1000);
    }
    
    /*if(sfr::serial::update_servos)
    {
        digitalWrite(ledPin, HIGH);
        delay(1000);
    }
    else if(true)
    {
        digitalWrite(ledPin, LOW);
        delay(2000);
        digitalWrite(ledPin, HIGH);
        delay(2000);
    }*/
};