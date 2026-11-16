"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class HondaFlagsSP(IntFlag):
  NIDEC_HYBRID = 1
  EPS_MODIFIED = 2
  HYBRID_ALT_BRAKEHOLD = 4
  # Nidec longitudinal mode for serial steering cars (see nidec_long/). Neither bit set = "mapped" (default)
  NIDEC_LONG_STOCK = 8
  NIDEC_LONG_ADAPTIVE = 16


class HondaSafetyFlagsSP:
  NIDEC_HYBRID = 1
  GAS_INTERCEPTOR = 2
  NIDEC_SERIAL_STEERING = 4  # allow STEERING_CONTROL on the camera bus for an LKAS/CAN adapter board
