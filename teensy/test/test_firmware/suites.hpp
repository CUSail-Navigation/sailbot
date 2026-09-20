#pragma once

/**
 * This file is the test suite registry. It acts almost like a header file for \code main.cpp\endcode.
 *
 * PlatformIO builds each directory under \code test/\endcode into one binary with a single \code main()\endcode, so all
 * of these tests link together. They are still split across files by topic; each file keeps its own tests file-local,
 * \code static\endcode, and exposes just the one function below, which \code main.cpp\endcode calls.
 */
void run_serial_framing_tests();
void run_servo_control_tests();
void run_telemetry_tests();
void run_anemometer_tests();
