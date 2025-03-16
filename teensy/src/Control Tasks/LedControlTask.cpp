#include "LedControlTask.hpp"

LedControlTask::LedControlTask()
{
    LED_PIN = 14;
    pinMode(LED_PIN, OUTPUT);
}

void LedControlTask::execute()
{
    Serial.println("here");
    if(Serial.available() > 0){
        Serial.println("reached 12");
        digitalWrite(LED_PIN, HIGH);
        delay(1000);
    }
    else if(sfr::serial::update_servos && Serial.available())
    //default state blinks if packets are received correctly (update_servos is true) and serial is available
    {
        //turn off boolean when light on
        digitalWrite(LED_PIN, LOW);
        delay(2000);
        digitalWrite(LED_PIN, HIGH);
        delay(2000);
    }
    else if(Serial.available() == 0) //FIX ME NOT WORKING
    //if serial is not available LED stays off
    {
        digitalWrite(LED_PIN, LOW);
        delay(1000);
    }
    else if(!sfr::serial::update_servos) //FIX ME NOT WORKING
    //if packets are not received correctly LED stays on
    {
        digitalWrite(LED_PIN, HIGH);
        delay(200);
    }
    else
    //in any other case LED blinks rapidly
    {
        digitalWrite(LED_PIN, LOW);
        delay(500);
        digitalWrite(LED_PIN, HIGH);
        delay(500);
    }
};  