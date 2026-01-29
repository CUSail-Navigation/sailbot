#include "AnemometerMonitor.hpp"

AnemometerMonitor* AnemometerMonitor::_instance = nullptr;

AnemometerMonitor::AnemometerMonitor()
{
    pinMode(constants::anemometer::ANEMOMETER_PIN_SPEED, INPUT_PULLUP);
    pinMode(constants::anemometer::ANEMOMETER_PIN_DIR, INPUT);

    _instance = this;

    last_pulse_time = 0;
    wind_speed = 0;
    elapsed_time = 0;
    have_period = false;

    attachInterrupt(digitalPinToInterrupt(constants::anemometer::ANEMOMETER_PIN_SPEED), 
                    windSpeedISR, 
                    FALLING);
}


void AnemometerMonitor::windSpeedISR() {
    if (_instance != nullptr) {
        unsigned long now = millis();
        _instance->last_pulse_ms_isr = now; 
        if (_instance->last_pulse_time == 0) {
            _instance->last_pulse_time = now;
        } else {
            _instance->elapsed_time = now - _instance->last_pulse_time;
            _instance->last_pulse_time = now;
            _instance->have_period = true;
        }
    }
}

void AnemometerMonitor::execute()
{  
    //WIND DIR
    anemometer::wind_angle = ((0.97826))*((360*analogRead(constants::anemometer::ANEMOMETER_PIN_DIR))/1000);//0.97826 = 360/368
    //adjusted value converts 368 to 360
    //Serial.println(anemometer::wind_angle); //print for testing

    noInterrupts();
    period_ms = elapsed_time;
    last_ms = last_pulse_ms_isr;
    ok = have_period;
    interrupts();
    

    if (!ok || last_ms == 0 || (millis() - last_ms) > 2000) {
        anemometer::wind_speed = 0.0f;
    } else if (period_ms > 0) {
        float T = period_ms / 1000.0f;
        anemometer::wind_speed = 2.25f / T;
    } else {
        anemometer::wind_speed = 0.0f;
    }
    Serial.println(anemometer::wind_speed); //print for testing
}
