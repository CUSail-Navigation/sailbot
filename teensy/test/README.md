# Teensy Tests

Code logic unit testing for the Teensy firmware that runs locally, with no need to have a Teensy plugged in.

_Note that to run tests locally, having a local C++17 compiler is required: typically `g++` or `clang++` on Linux/macOS, 
and MinGW/MSVC on Windows._

---


## Running Tests
_Make sure you have an internet connection the first time you run these tests: PlatformIO will auto-download the 
`native` platform and the `Unity` test framework._

Included in the `test/` directory is a script (`run_tests.sh`) that you can use to run the entire test suite. From the
project root directory, `teensy/`, you can run:  
```bash
./test/run_tests.sh
```
The less robust but functional raw command is simply:
```bash 
pio test -e native
```

### IDE integration
It is also possible to run the test with one click in an editor, without using the command line.
- VSCode: PlatformIO Icon (in sidebar) → Project Tasks → native → Advanced → Test
-  CLion: create a run configuration for a shell script. Set "script path" to `test/run_tests.sh` and 
  "working directory" to the project root `teensy/`.

### Troubleshooting
- "`Error: could not find the 'pio' command`": PlatformIO Core is not installed, or is not on `PATH` (`run_tests.sh` 
  already falls back to `~/.platformio/penv/bin/pio`).
- A compiler error (mentioning `Arduino.h` or `Servo.h`):  new source code is using functions that the mocks do not yet 
  implement. Add these to the appropriate file in  (`test/mocks/`); they only cover what the firmware currently calls. 
- A test passes alone but fails in the suite: almost always leaked global state. Check that `setUp()` calls
`reset_all()`, and that any new SFR field is reset in `reset_sfr()`.


## Organization and Adding Tests
### Structure 
All tests build into a single suite, but are split across one file per topic in `test/test_firmware/`. 
- PlatformIO runs each folder under `test/` as its own binary needing its own `main()`. 
- One folder thus means one build/link/process. Each topic exposes a single `run_<topic>_tests()` called by `main.cpp`.
- For more information about what is covered by the full suite, read the documentation in each topic file.

There are also a bunch of shared useful functions organized in the `test_firmware/test_support.h` file.

### Extending Existing Topics
Add a `static void test_something()` function to the relevant file, put your testing logic in this function, and add a 
matching `RUN_TEST()` line in that file's `run_<topic>_tests()` function.

### Adding New Topics
Create a new file (`test/test_firmware/<topic>.cpp`), following the templates of the topics already in the 
`test/test_firmware/` directory. Make sure to declare its run function in `suites.hpp` and call it from `main.cpp` to 
have the tests actually run.

--- 


## Mocks
The firmware needs and uses `Arduino.h` and `Servo.h` for functions like `millis()`, `Serial`, `analogRead()`.
Those only exist when cross-compiling for the Teensy, implying that tests could only run with hardware attached.

To avoid this issue and make it possible to test locally, `test/mocks/` contains stand-ins named exactly `Arduino.h` and
`Servo.h`. This means `#include <Arduino.h>` in source code resolves to the **mock** when building tests and to the 
**genuine Teensy header** when building firmware. In both cases, the production code is compiled unmodified: nothing in 
`src/` is aware that the  fakes for testing exist.


Some advantages that the mocks bring over real hardware:
  - A clock we can manipulate by hand: timeouts become instant and deterministic rather than requiring real waiting.
  - Serial ports to feed bytes into: `Serial.mock_rx({...})` queues bytes as if the Jetson/XBee had sent them, and
    `Serial.mock_tx()` shows exactly what the firmware wrote back. 
  - We can see what each servo was commanded to do, including whether it moved at all, without needing actual servos.
  - All mocks are header-only and use C++17 `inline` variables: there is no separate `.cpp` file to keep in sync and no
    multiple-definition problems regardless of how many files include them.
---


## Additional Information
1. **PlatformIO supports testing with a board plugged in, which is more faithful, but less practical.** 
   - This suite tests code logic and does not require the actual boat or Teensy board.
   - Actual hardware-specific behavior (servo movement, real Serial communication) is *not* covered by these tests. 
     This test suite therefore cannot replace actual boat-testing.
     - `MainControlLoop` is untested: while the individual monitors and tasks are tested, the order they run in is not.
2. **For proper encapsulation, there is no direct unit test of a private method anywhere in this suite: private methods 
     are covered indirectly. This suite is more about testing functionality class by class.**
    - The workaround used instead is to reach private methods *through* the public `execute()` -- the `apply_commands()`
      method writes every value these helpers compute into the SFR, so a test can stage a command, call `execute()`, and
      read the computed PWM back out. 
    - The mock `Servo` class records what reaches each pin, confirming numbers actually get sent to the right servo. 
    - Some resulting consequences to be aware of:
      - Failures point appear at the wrong place (a broken `law_of_cos_map()` surfaces as a failing `execute()` 
        assertion; you have to work backwards to the error yourself).
      - Coverage is coupled to the caller (if `execute()` ever stops routing through one of these helpers, its tests 
        might pass quietly while the helper degrades).
3. **Convention: no magic numbers. Use `constants.hpp`.**
   - Everything in `constants.hpp` is boat calibration, which changes with a new boat. A test must never hardcode one 
     of these values, or it would have to be manually updated with every new boat.
   - Instead, the tests assert *relationships* that hold under any calibration. For example:
     - The minimum angle maps to `MIN_PULSE`; the maximum to `MAX_PULSE`.
       - Every servo is expected to be rigged so a larger commanded angle means a larger pulse width.
       - PWM output should never escape `[MIN_PULSE, MAX_PULSE]`.
     - A serial packet of exactly `<USB/RADIO>_BUFFER_LEN` payload bytes is accepted; anything shorter or longer is dropped.
4. **For more information about PlatformIO Unit Testing, vist this [link](https://docs.platformio.org/en/latest/advanced/unit-testing/index.html).**
