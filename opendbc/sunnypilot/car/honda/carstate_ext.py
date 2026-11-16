"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP
from opendbc.sunnypilot.car.honda.nidec_long import NidecLongMode, get_mode as get_nidec_long_mode


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    # used by the MDX ADAPTIVE longitudinal mode (nidec_long/adaptive.py)
    self.car_gas = 0.0
    self.car_gas_available = False
    self.engine_rpm = 0.0

  def update(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    # only read in ADAPTIVE mode, so the extra messages never affect canValid otherwise
    if get_nidec_long_mode(self.CP, self.CP_SP) == NidecLongMode.ADAPTIVE:
      # applied gas pedal (GAS_PEDAL 0x13C on the MDX); RPM from both messages, as MVL (ENGINE_DATA alone read 0 on the MDX)
      self.car_gas = cp.vl["GAS_PEDAL"]["CAR_GAS"]
      self.car_gas_available = True
      self.engine_rpm = max(cp.vl["ENGINE_DATA"]["ENGINE_RPM"], cp.vl["POWERTRAIN_DATA"]["ENGINE_RPM"])

    if self.CP_SP.flags & HondaFlagsSP.NIDEC_HYBRID:
      ret.accFaulted = bool(cp.vl["HYBRID_BRAKE_ERROR"]["BRAKE_ERROR_1"] or cp.vl["HYBRID_BRAKE_ERROR"]["BRAKE_ERROR_2"])
      ret.stockAeb = bool(cp_cam.vl["BRAKE_COMMAND"]["AEB_REQ_1"] and cp_cam.vl["BRAKE_COMMAND"]["COMPUTER_BRAKE_HYBRID"] > 1e-5)

    if self.CP_SP.flags & HondaFlagsSP.HYBRID_ALT_BRAKEHOLD:
      ret.brakeHoldActive = cp.vl["BRAKE_HOLD_HYBRID_ALT"]["BRAKE_HOLD_ACTIVE"] == 1
      ret.blockPcmEnable = ret.brakeHoldActive and not self.CP_SP.enableGasInterceptor

    if self.CP_SP.enableGasInterceptor:
      # Same threshold as panda, equivalent to 1e-5 with previous DBC scaling
      gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) // 2
      ret.gasPressed = gas > 492
