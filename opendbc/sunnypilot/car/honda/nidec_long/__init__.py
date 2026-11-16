"""
Pluggable Nidec longitudinal modes for the serial steering Acura MDX (ACURA_MDX_3G_SERIAL).

  STOCK    (0): sunnypilot's stock Nidec mapping (stepped PCM_SPEED map, stock kiV, 0.15 s delay)
  MAPPED   (1): default. PCM_SPEED = vEgo + 4 s * accel capped at the set speed, drag in the gas limit, half kiV
  ADAPTIVE (2): port of the MVL fork's adaptive Nidec controller (adaptive.py, see MVL_SYNC.md)

The mode comes from the openpilot param "HondaNidecLongMode" and is applied by apply_mode() from the openpilot side
(sunnypilot/selfdrive/car/honda_nidec_long.py) right after the car interface is created. Without that hook
(e.g. opendbc tests) the car runs MAPPED. Switch on the device with:  echo -n 2 > /data/params/d/HondaNidecLongMode
"""
from enum import IntEnum

from opendbc.car import structs
from opendbc.car.honda.values import SERIAL_STEERING, SerialSteeringLongitudinal
from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP


class NidecLongMode(IntEnum):
  STOCK = 0
  MAPPED = 1
  ADAPTIVE = 2


def supports_nidec_long_modes(CP: structs.CarParams) -> bool:
  return CP.carFingerprint in SERIAL_STEERING


def get_mode(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> NidecLongMode:
  if not supports_nidec_long_modes(CP):
    return NidecLongMode.STOCK
  if CP_SP.flags & HondaFlagsSP.NIDEC_LONG_ADAPTIVE:
    return NidecLongMode.ADAPTIVE
  if CP_SP.flags & HondaFlagsSP.NIDEC_LONG_STOCK:
    return NidecLongMode.STOCK
  return NidecLongMode.MAPPED


def apply_mode(CP: structs.CarParams, CP_SP: structs.CarParamsSP, mode: int) -> NidecLongMode:
  """Select a mode: sets the CP_SP flag and the openpilot-side longitudinal tuning that goes with it.
  Must run before CarParams are published (card.py does this via setup_interfaces)."""
  if not supports_nidec_long_modes(CP):
    return NidecLongMode.STOCK
  try:
    mode = NidecLongMode(int(mode))
  except (TypeError, ValueError):
    mode = NidecLongMode.MAPPED

  CP_SP.flags &= ~(HondaFlagsSP.NIDEC_LONG_STOCK | HondaFlagsSP.NIDEC_LONG_ADAPTIVE)
  if mode == NidecLongMode.STOCK:
    CP_SP.flags |= HondaFlagsSP.NIDEC_LONG_STOCK.value
    CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV = [0., 5., 35.], [1.2, 0.8, 0.5]
    CP.longitudinalActuatorDelay = 0.15
  elif mode == NidecLongMode.ADAPTIVE:
    CP_SP.flags |= HondaFlagsSP.NIDEC_LONG_ADAPTIVE.value
    # MVL runs openpilot's longitudinal PID as pure feedforward on Nidec and corrects in the car controller instead
    CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV = [0.], [0.]
    CP.longitudinalActuatorDelay = 0.15
  else:
    L = SerialSteeringLongitudinal
    CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV = L.KI_BP, L.KI_V
    CP.longitudinalActuatorDelay = 0.5
  return mode
