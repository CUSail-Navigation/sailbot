#include "LedControlTask.hpp"

LedControlTask::LedControlTask() : LED_PIN(constants::led::LED_PIN) {
    pinMode(LED_PIN, OUTPUT);
}

/**
 * Blinks the status LED to indicate the current connectivity/data state of the boat.
 */
void LedControlTask::execute() const {
    const bool update_servos = sfr::serial::update_servos_radio || sfr::serial::update_servos_usb;
    if (Serial.available() && !update_servos) {
        digitalWrite(LED_PIN, LOW);
        delay(2000);
        digitalWrite(LED_PIN, HIGH);
        delay(2000);
    }
    else if (update_servos) {
        digitalWrite(LED_PIN, HIGH);
        delay(1000);
    }
    else if (!Serial.available()) {
        digitalWrite(LED_PIN, LOW);
        delay(1000);
    }
    else {
        digitalWrite(LED_PIN, LOW);
        delay(500);
        digitalWrite(LED_PIN, HIGH);
        delay(500);
    }
}
