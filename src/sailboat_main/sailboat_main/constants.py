"""
BOAT CONSTANTS.

Use these to avoid "magic numbers" in code, and to keep everything specific to a given boat centrally located.

Note that the ``dataclass`` import and type usage in each inner class works to make each constant truly immutable.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class _Physical:
    """
    Physical parameters of the boat.
    These constants concern the 2025-2026 season boat. (Update this every year).
    """
    RUDDER_MIN_ANGLE: int = -25
    RUDDER_MAX_ANGLE: int = 25

    MAINSAIL_MIN_ANGLE: int = 0
    MAINSAIL_MAX_ANGLE: int = 90

    JIB_MIN_ANGLE: int = 0
    JIB_MAX_ANGLE: int = 90
    # Flag for which side to set the jib on.
    JIB_SIDE_PORT: int = 0
    JIB_SIDE_STB: int = 1


@dataclass(frozen=True)
class _Wind:
    """
    Conventions for interpreting wind sensor data.
    """
    NO_GO_CENTER: int = 180
    NO_GO_WIDTH: int = 90  # (2024-2025)


@dataclass(frozen=True)
class _Serial:
    """
    Parameters for Serial communication with the Teensy.
    """
    TX_START_FLAG: int = 0xFF
    TX_END_FLAG: int = 0xEE
    TX_PERIOD_MS: int = 500
    TX_PACKET_LEN: int = 7

    RX_START_FLAG: int = 0xFF
    RX_END_FLAG: int = 0xEE
    RX_PERIOD_MS: int = 500

    BAUD_RATE: int = 9600


PHYSICAL = _Physical()
WIND = _Wind()
SERIAL = _Serial()
