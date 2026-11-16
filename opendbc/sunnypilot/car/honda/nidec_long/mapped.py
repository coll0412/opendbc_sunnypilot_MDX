"""
MAPPED mode (default): simplified MVL-style mapping, no learning. Unchanged from the version first driven (2606a973).
"""
import numpy as np

from opendbc.car.honda.values import SerialSteeringLongitudinal


def mapped_pcm_commands(v_ego: float, accel: float, hud_control, max_accel: float, gas_max: int) -> tuple[float, int]:
  """Returns (pcm_speed m/s, pcm_accel wire units)."""
  L = SerialSteeringLongitudinal
  # continuous PCM speed target instead of the stepped map (which sent 0 or ~vEgo+5 m/s for small accel changes)
  pcm_speed = float(np.clip(v_ego + L.SPEED_LEAD_TIME * accel, 0.0, 100.0))
  # the PCM is itself a speed servo: never ask for more than just above the set speed
  if hud_control.speedVisible and hud_control.setSpeed > 0.:
    pcm_speed = min(pcm_speed, hud_control.setSpeed + L.SET_SPEED_MARGIN)
  # gas limit includes aerodynamic drag, so holding speed doesn't depend on the PID integrator
  drag = float(np.interp(v_ego, L.DRAG_BP, L.DRAG_V))
  pcm_accel = int(np.clip(((accel + drag) / 1.44) / max_accel, 0.0, 1.0) * gas_max)
  return pcm_speed, pcm_accel
