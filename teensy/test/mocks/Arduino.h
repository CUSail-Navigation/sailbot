#pragma once
#include <cstdint>
#include <cstring>
#include <cstddef>
#include <cstdio>
#include <cmath>
#include <deque>
#include <vector>
#include <initializer_list>
#include <type_traits>


// Pin modes and logic levels.
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2


// Simulated clock (firmware's packet timeouts are driven entirely by millis(), so tests advance this manually).
inline uint32_t g_mock_millis = 0;

/** Returns the current fake time, in milliseconds. */
inline uint32_t millis() {
    return g_mock_millis;
}

/** Returns the current fake time in microseconds, derived from the millisecond clock. */
inline uint32_t micros() {
    return g_mock_millis * 1000;
}

/** Jumps the fake clock to an absolute time, \code ms\endcode. */
inline void mock_set_millis(const uint32_t ms) {
    g_mock_millis = ms;
}

/** Moves the fake clock forward by \code ms\endcode. Use this to trip \code RX_PACKET_TIMEOUT_MS\endcode. */
inline void mock_advance_millis(const uint32_t ms) {
    g_mock_millis += ms;
}

/** Simulates a delay without sleeping: only moves the fake clock forward by \code ms\endcode. */
inline void delay(const uint32_t ms) {
    g_mock_millis += ms;
}

/** Accept and ignore this call -- the fake clock has millisecond resolution and cannot represent this. */
inline void delayMicroseconds(const uint32_t) {}

/**  Accept and ignore this call -- there is no background work to yield to on the host. */
inline void yield() {}


// Digital/analog IO. Writes are recorded so a test can assert on them; reads return whatever the test staged.
inline constexpr size_t MOCK_PIN_COUNT = 64;

inline int g_mock_pin_modes[MOCK_PIN_COUNT] = {};
inline int g_mock_pin_levels[MOCK_PIN_COUNT] = {};
inline int g_mock_analog_values[MOCK_PIN_COUNT] = {};

/** Records the mode that a pin was configured to. */
inline void pinMode(const uint8_t pin, const int mode) {
    if (pin < MOCK_PIN_COUNT) g_mock_pin_modes[pin] = mode;
}

/** Records a logic level written to a pin. */
inline void digitalWrite(const uint8_t pin, const int level) {
    if (pin < MOCK_PIN_COUNT) g_mock_pin_levels[pin] = level;
}

/** Returns the last logic level written to a pin, or 0 if it was never written. */
inline int digitalRead(const uint8_t pin) {
    return pin < MOCK_PIN_COUNT ? g_mock_pin_levels[pin] : 0;
}

/** Returns the analog value a test staged for a pin, or 0 if none was staged. */
inline int analogRead(const uint8_t pin) {
    return pin < MOCK_PIN_COUNT ? g_mock_analog_values[pin] : 0;
}

/** Records an analog value written to a pin, in the same slot \code analogRead()\endcode reads back from. */
inline void analogWrite(const uint8_t pin, const int value) {
    if (pin < MOCK_PIN_COUNT) g_mock_analog_values[pin] = value;
}

/** Stages the value \code analogRead(pin)\endcode will return, such as a raw anemometer ADC reading. */
inline void mock_set_analog(const uint8_t pin, const int value) {
    if (pin < MOCK_PIN_COUNT) g_mock_analog_values[pin] = value;
}

/** Reads back the last level written by \code digitalWrite(pin)\endcode, for asserting on pin state. */
inline int mock_pin_level(const uint8_t pin) {
    return pin < MOCK_PIN_COUNT ? g_mock_pin_levels[pin] : 0;
}


// Specific implementation for the map() method.
/**
 * A faithful port of the Teensy 4 core's \code map()\endcode -- NOT the simpler classic Arduino one, which leaves out
 * the round-off correction below and so returns different values. This is one of the few Teensy functions whose
 * internals have to be matched exactly here, because \code rudder_to_pwm()\endcode maps angles through it.
 */
template <class T, class A, class B, class C, class D>
long map(T _x, A _in_min, B _in_max, C _out_min, D _out_max,
         typename std::enable_if<std::is_integral<T>::value>::type* = 0) {
    long x = _x, in_min = _in_min, in_max = _in_max, out_min = _out_min, out_max = _out_max;

    const long in_range = in_max - in_min;
    const long out_range = out_max - out_min;
    if (in_range == 0) return out_min + out_range / 2;

    long num = (x - in_min) * out_range;
    if (out_range >= 0) num += in_range / 2; // Round off towards zero.
    else num -= in_range / 2;

    const long result = num / in_range + out_min;

    // Teensy's fix for "a strange behaviour with negative numbers" (ArduinoCore-API issue #51).
    if (out_range >= 0) {
        if (in_range * num < 0) return result - 1;
    } else {
        if (in_range * num >= 0) return result + 1;
    }
    return result;
}


// Serial ports: stand in for both USB port to the Jetson and the hardware UART to the XBee.
class FakeStream {
    std::deque<uint8_t> rx_queue;
    std::vector<uint8_t> tx_log;

public:
    /** Accept and ignore this call -- there is no real baud rate to configure. */
    void begin(unsigned long /*baud*/) {}

    /**  Accept and ignore this call -- there is no real port to close. */
    void end() {}

    /** Returns how many queued bytes are waiting to be read. */
    int available() {
        return static_cast<int>(rx_queue.size());
    }

    /** Pops one queued byte, or returns -1 when empty (matching the real Stream contract). */
    int read() {
        if (rx_queue.empty()) return -1;
        const int byte = rx_queue.front();
        rx_queue.pop_front();
        return byte;
    }

    /** Returns the next queued byte without consuming it, or -1 when empty. */
    int peek() const {
        return rx_queue.empty() ? -1 : rx_queue.front();
    }

    /** Appends one byte to the transmit log, reporting the single byte written. */
    size_t write(const uint8_t byte) {
        tx_log.push_back(byte);
        return 1;
    }

    /** Appends \code size\endcode bytes to the transmit log, reporting how many were written. */
    size_t write(const uint8_t* buffer, const size_t size) {
        tx_log.insert(tx_log.end(), buffer, buffer + size);
        return size;
    }

    /**  Accept and ignore this call -- writes to the log are never buffered. */
    void flush() {}

    /** Discards printed output; exists only so that debug prints in the firmware still compile under test. */
    template <typename T> size_t print(const T&) {
        return 0;
    }

    /** Discards printed output, as \code print()\endcode does. */
    template <typename T> size_t println(const T&) {
        return 0;
    }

    /** Discards a bare newline, as \code print()\endcode does. */
    size_t println() {
        return 0;
    }


    // Test-only hooks below (how a test feeds bytes in and reads back what the firmware sent out).
    /** Queues bytes as if the peer had just transmitted them. */
    void mock_rx(const std::initializer_list<uint8_t> bytes) {
        for (const uint8_t byte : bytes) rx_queue.push_back(byte);
    }

    /** Queues \code count\endcode bytes from a buffer, for feeding in a packet built at runtime. */
    void mock_rx(const uint8_t* bytes, const size_t count) {
        for (size_t i = 0; i < count; ++i) rx_queue.push_back(bytes[i]);
    }

    /** Queues a single byte, for building up a partial packet one byte at a time. */
    void mock_rx_byte(const uint8_t byte) { rx_queue.push_back(byte); }

    /** Returns everything the firmware has written to this port since the last reset. */
    const std::vector<uint8_t>& mock_tx() const { return tx_log; }

    /** Returns how many queued bytes the firmware has not consumed yet. */
    size_t mock_rx_pending() const { return rx_queue.size(); }

    /** Empties both the receive queue and the transmit log, so that the next test starts clean. */
    void mock_clear() {
        rx_queue.clear();
        tx_log.clear();
    }
};

/** The Jetson link (USB serial): read by \code USBSerialMonitor\endcode, written by \code TelemetryControlTask\endcode. */
inline FakeStream Serial;

/** The XBee radio link (hardware UART on pins 7/8): read by \code RadioSerialMonitor\endcode. */
inline FakeStream Serial2;
