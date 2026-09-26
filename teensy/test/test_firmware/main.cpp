#include "test_support.h"
#include "suites.hpp"

/**
 * This method is the entry point for the Teensy unit tests.
 *
 * The test framework \code Unity\endcode requires exactly one \code main()\endcode, one \code setup()\endcode, and one
*  \code tearDown()\endcode per binary, so they are centrally organized here. The function \code setup()\endcode runs
*  before every test in every topic (wipes the SFR and mocks so no test can inherit state from whatever ran before it).
 */
int main(int, char**) {
    UNITY_BEGIN();

    run_serial_framing_tests();
    run_servo_control_tests();
    run_telemetry_tests();
    run_anemometer_tests();

    return UNITY_END();
}

void setUp() {
    reset_all();
}

void tearDown() {}
