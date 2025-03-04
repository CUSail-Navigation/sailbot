#include "LedControlTask.hpp"

LedControlTask::LedControlTask()
{
    LED_PIN = constants::led::LED_PIN;
    pinMode(LED_PIN, OUTPUT);
}

void LedControlTask::execute()
{
    if(!sfr::serial::update_servos && Serial.available())
    {
        digitalWrite(LED_PIN, LOW);
        delay(2000);
        digitalWrite(LED_PIN, HIGH);
        delay(2000);
    }
    else if(sfr::serial::update_servos)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(1000);
    }
    else if(!Serial.available())
    {
        digitalWrite(LED_PIN, LOW);
        delay(1000);
    }
    else{
        digitalWrite(LED_PIN, LOW);
        delay(500);
        digitalWrite(LED_PIN, HIGH);
        delay(500);
    }
};  