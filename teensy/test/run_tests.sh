#!/usr/bin/env bash
# A script to actually run the Teensy unit test suite. Some options:
#   ./run_tests.sh        # run everything
#   ./run_tests.sh -v     # verbose (shows the compiler command lines)
# Note that anything you pass is forwarded to `pio test`.


set -euo pipefail       # Error handling.
cd "$(dirname "$0")/.." # Script lives in test/, but pio has to run from the project root, where platformio.ini is.

# PlatformIO setup/verification. Note that it is often installed to ~/.platformio/penv/bin rather than onto PATH.
if command -v pio > /dev/null 2>&1; then
    PIO=pio
elif [ -x "$HOME/.platformio/penv/bin/pio" ]; then
    PIO="$HOME/.platformio/penv/bin/pio"
else
    echo "Error: could not find the 'pio' command." >&2
    echo "Install PlatformIO Core: https://docs.platformio.org/en/latest/core/installation/index.html" >&2
    exit 127
fi

# Actually run the tests. "-e native" pins this to host-side environment (never tries to talk to a Teensy).
exec "$PIO" test -e native "$@"
