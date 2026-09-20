#pragma once
#include "Arduino.h"

// Per-pin records of what each servo was commanded to do, so tests can observe servo activity.
inline int g_mock_servo_last_write[MOCK_PIN_COUNT] = {};
inline int g_mock_servo_write_count[MOCK_PIN_COUNT] = {};

/** Clears both records above, so one test cannot see the servo commands issued by the previous one. */
inline void mock_reset_servos() {
    for (size_t pin = 0; pin < MOCK_PIN_COUNT; ++pin) {
        g_mock_servo_last_write[pin] = 0;
        g_mock_servo_write_count[pin] = 0;
    }
}


// Stand-in for the Arduino Servo class.
class Servo {
    int attached_pin = -1;
    int last_value = 0;
    bool is_attached = false;

public:
    /** Records which pin this servo drives, ignoring the pulse bounds (nothing is able to read them back). */
    void attach(const int pin, const int /*min_pulse*/, const int /*max_pulse*/) {
        attach(pin);
    }

    /** Records which pin this servo drives. */
    void attach(const int pin) {
        attached_pin = pin;
        is_attached = true;
    }

    /** Marks this servo as no longer driving its pin. */
    void detach() {
        is_attached = false;
    }

    /** Returns whether \code attach()\endcode has been called without a later \code detach()\endcode. */
    bool attached() const {
        return is_attached;
    }

    /** Records a commanded pulse width, both on this instance and against the pin it is attached to. */
    void write(const int value) {
        last_value = value;
        if (attached_pin >= 0 && static_cast<size_t>(attached_pin) < MOCK_PIN_COUNT) {
            g_mock_servo_last_write[attached_pin] = value;
            ++g_mock_servo_write_count[attached_pin];
        }
    }

    /** Records a pulse width given explicitly in microseconds (identical to \code write()\endcode for this mock). */
    void writeMicroseconds(const int value) {
        write(value);
    }

    /** Returns the last value passed to \code write()\endcode. */
    int read() const {
        return last_value;
    }
};
