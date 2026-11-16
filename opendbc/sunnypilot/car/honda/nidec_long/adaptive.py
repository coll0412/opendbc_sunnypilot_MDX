"""
ADAPTIVE mode: port of the MVL fork's adaptive Nidec longitudinal controller (mvl-boston/openpilot, honda
carcontroller.py, opendbc 3278c6bef). See MVL_SYNC.md for the source mapping and how to re-sync.

What it does, in short (MVL's comments carry the field evidence):
- openpilot's longitudinal PID runs as pure feedforward (kiV = 0, set by apply_mode); a local integrator that can only
  REDUCE accel (pos_limit 0) corrects overshoot, so there is no positive wind-up
- PCM_SPEED = vEgo + speedfactor * accel + speedalpha (two speed bands), learned against the measured response
- PCM_GAS from accel + hill + learned wind drag, scaled by a learned gas factor, then through a feedforward model of
  the PCM's own smoothing (average_factor), with a rise clip and recovery windows after brake/override/launch
- launch governor from standstill, extra brake PID, learned saturation accel
- learned values persist through an injected store (one JSON param on the device)

Not ported: MVL's lateral latFactors learner, Bosch paths, actuator-field logging overrides.
"""
import math
from dataclasses import dataclass

import numpy as np

import json

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, DT_CTRL, rate_limit, structs
from opendbc.car.carlog import carlog
from opendbc.car.common.pid import PIDController
from opendbc.sunnypilot.car.honda.nidec_long.store import load_seed, load_state, save_state

LongCtrlState = structs.CarControl.Actuators.LongControlState

SAVE_INTERVAL_FRAMES = 6000  # 60 s, as MVL

# learned values: default, (min, max) -- MVL's defaults and runtime clips
LEARNED = {
  "average_factor":       (0.5, (0.02, 1.0)),
  "brake_pid_factor":     (0.4, (0.0, 4.0)),
  "creep_factor":         (1.0, (0.0, 3.0)),
  "gas_alpha":            (0.0, (-3.0, 3.0)),
  "gasfactor":            (1.0, (0.1, 5.0)),
  "gasfactor_low":        (1.0, (0.1, 5.0)),
  "windfactor":           (1.0, (0.1, 3.0)),
  "speedfactor":          (4.0, (0.01, 99.0)),
  "speedalpha":           (0.0, (-10.0, 10.0)),
  "speedfactor_low":      (4.0, (0.01, 99.0)),
  "speedalpha_low":       (0.0, (-10.0, 10.0)),
  "sat_accel":            (0.9, (0.1, 1.9)),
  "car_gas_per_pcm_gas":  (0.3, (0.00001, 5.0)),
  "dv_launch":            (2.8, (1.0, 8.0)),
  "gas_launch":           (110.0, (40.0, 198.0)),
  "dv_break":             (6.0, (2.0, 12.0)),
}


def compute_gb_honda_nidec(accel, speed, creep_factor):
  creep_brake = 0.0
  creep_speed = 2.3
  creep_brake_value = 0.15
  if speed < creep_speed:
    creep_brake = (creep_speed - speed) / creep_speed * creep_brake_value
  gb = float(accel) / 4.8 - creep_brake * creep_factor
  creep_impact = -creep_brake
  return float(np.clip(-gb, 0.0, 1.0)), creep_impact


def actuator_hysteresis(brake, braking, brake_steady):
  brake_hyst_on = 0.02    # to activate brakes exceed this value
  brake_hyst_off = 0.005  # to deactivate brakes below this value
  brake_hyst_gap = 0.01   # don't change brake command for small oscillations within this value
  if (brake < brake_hyst_on and not braking) or brake < brake_hyst_off:
    brake = 0.
  braking = brake > 0.
  if brake == 0.:
    brake_steady = 0.
  elif brake > brake_steady + brake_hyst_gap:
    brake_steady = brake - brake_hyst_gap
  elif brake < brake_steady - brake_hyst_gap:
    brake_steady = brake + brake_hyst_gap
  return brake_steady, braking, brake_steady


def brake_pump_hysteresis(apply_brake, apply_brake_last, last_pump_ts, ts):
  pump_on = False
  if apply_brake > apply_brake_last or (ts - last_pump_ts > 20. and apply_brake > 0):
    last_pump_ts = ts
  if ts - last_pump_ts < 0.2 and apply_brake > 0:
    pump_on = True
  return pump_on, last_pump_ts


@dataclass
class AdaptiveOutput:
  pcm_speed: float = 0.0     # m/s, ACC_HUD PCM_SPEED
  pcm_gas: int = 0           # wire units, ACC_HUD PCM_GAS
  set_me_x01: int = 1        # ACC_HUD SET_ME_X01 ("1 gives power")
  apply_brake: int = 0       # BRAKE_COMMAND COMPUTER_BRAKE, valid on even frames
  pump_on: bool = False
  accel: float = 0.0         # corrected accel actually targeted (m/s^2)
  gas_target: float = 0.0    # pre-feedforward gas target / NIDEC_GAS_MAX, for the actuator output field


class NidecAdaptiveLongitudinal:
  def __init__(self, CP, params, store=None):
    self.CP = CP
    self.params = params  # CarControllerParams
    self.store = store
    self.frame = 0

    s, source = load_state(store, {k: d for k, (d, _) in LEARNED.items()}, {k: b for k, (_, b) in LEARNED.items()}, load_seed())
    # recorded in the drive log (rlog logMessage), so learned values can be recovered from any drive
    carlog.warning(json.dumps({"event": "honda_nidec_long_loaded", "source": source, "values": s}))
    s["sat_accel"] = min(s["sat_accel"], self.params.NIDEC_ACCEL_MAX - 0.1)
    for k, v in s.items():
      setattr(self, k, float(v))
    self.average_factor = max(0.02, self.average_factor)  # same floor as the learner

    self.gasfactor_before_gasmax = self.gasfactor_nomaxspeed = self.gasfactor
    self.gasfactor_low_before_gasmax = self.gasfactor_low_nomaxspeed = self.gasfactor_low
    self.gas_alpha_nomaxspeed = self.gas_alpha
    self.windfactor_before_gasmax = self.windfactor_before_brake = self.windfactor
    self.brake_pid_factor_non_lowspeed = self.brake_pid_factor

    self.pitch = 0.0
    self.nidec_pid_factor = 0.0
    self.nidec_pid = PIDController(k_p=([0], [0]), k_i=([0.01, 5., 35.], [1.2, 0.8, 0.5]), k_f=1,
                                   pos_limit=0., neg_limit=self.params.NIDEC_ACCEL_MIN)
    self.brake_pid = PIDController(k_p=0.0, k_i=2.0, pos_limit=4.0, neg_limit=0, rate=50)
    self.brake_pid.i = self.brake_pid_factor_non_lowspeed

    self.prior_gas_average = 0.0
    self.average_factor_sens = 0.0
    self.sat_deficit_frames = self.sat_excess_frames = 0
    self.new_accel = 0  # wire PCM_GAS (MVL name)
    self.accel = 0.0

    # brake path
    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.apply_brake_last = 0
    self.last_pump_ts = 0.
    self.pump_on = False

    # launch governor / recovery windows
    self.launch_active = False
    self.gas_recovery_ticks = 0
    self.gas_pressed_prev = False
    self.long_active_prev = False
    self.launch_ticks = 0
    self.launch_release_tick = -1
    self.launch_move_tick = -1
    self.launch_lurch_sum = 0.0
    self.launch_lurch_n = 0
    self.launch_err_sum = 0.0
    self.launch_err_n = 0
    self.launch_ceiling_ticks = 0

  def learned_state(self) -> dict:
    s = {k: getattr(self, k) for k in LEARNED}
    # persist the "not near max speed" values, as MVL does
    s["gas_alpha"] = self.gas_alpha_nomaxspeed
    s["gasfactor"] = self.gasfactor_nomaxspeed
    s["gasfactor_low"] = self.gasfactor_low_nomaxspeed
    s["brake_pid_factor"] = self.brake_pid_factor_non_lowspeed
    return s

  def update(self, CC, CS, frame: int) -> AdaptiveOutput:
    P = self.params
    out = CS.out
    actuators = CC.actuators
    car_gas_available = getattr(CS, "car_gas_available", False)
    car_gas = getattr(CS, "car_gas", 0.0)
    engine_rpm = getattr(CS, "engine_rpm", 0.0)

    if len(CC.orientationNED) == 3:
      self.pitch = CC.orientationNED[1]
    hill_brake = math.sin(self.pitch) * ACCELERATION_DUE_TO_GRAVITY

    # wind brake from air resistance decel at high speed
    wind_brake = float(np.interp(out.vEgo, [0.0, 2.3, 35.0], [0.001, 0.002, 0.15])) * self.windfactor  # not in m/s2 units

    prior_windfactor = self.windfactor
    if CC.longActive:
      if (actuators.longControlState in (LongCtrlState.pid, LongCtrlState.stopping)) and \
         (out.vEgo > 1e-5 or actuators.accel > 1e-5) and (not out.stockAeb) and (not out.gasPressed):
        old_i = self.nidec_pid.i
        self.nidec_pid_factor = float(self.nidec_pid.update(error=actuators.accel - out.aEgo, speed=out.vEgo))
        if actuators.accel >= 1e-5 and out.vEgo < 1.0:  # don't drop PID at launch lurch
          self.nidec_pid_factor = self.nidec_pid.i = max(old_i, self.nidec_pid.i)
        self.accel = actuators.accel + self.nidec_pid_factor
        adjust_accel = self.accel + hill_brake

        gas_error = self.accel - out.aEgo
        wind_learn_speed = 1000
        wind_adjust = 1 + wind_brake / wind_learn_speed
        self.windfactor = float(np.clip(self.windfactor * (wind_adjust if (gas_error > 0) else 1.0 / wind_adjust), 0.1, 3.0))
        gas_pedal_force = self.accel
        if gas_pedal_force <= 0.0:  # don't reduce windfactor while braking, allow increases
          self.windfactor = max(self.windfactor, self.windfactor_before_brake)
        else:
          self.windfactor_before_brake = self.windfactor
        if out.vEgo < out.cruiseState.speed - 2.:
          # drop to max values when not near speed limit
          self.gasfactor = self.gasfactor_nomaxspeed
          self.gasfactor_low = self.gasfactor_low_nomaxspeed
          self.gas_alpha = self.gas_alpha_nomaxspeed
        if gas_pedal_force >= P.BOSCH_ACCEL_MAX:
          # don't increase gasfactor nor windfactor at accel max, allow decreases
          self.gasfactor = min(self.gasfactor, self.gasfactor_before_gasmax)
          self.gasfactor_low = min(self.gasfactor_low, self.gasfactor_low_before_gasmax)
          self.windfactor = min(self.windfactor, self.windfactor_before_gasmax)
        else:
          self.gasfactor_before_gasmax = self.gasfactor
          self.gasfactor_low_before_gasmax = self.gasfactor_low
          self.windfactor_before_gasmax = self.windfactor
      else:
        self.accel = actuators.accel + self.nidec_pid_factor
        adjust_accel = self.accel + hill_brake

      if (out.brakePressed or out.gasPressed or out.vEgo < 1e-5) and (self.nidec_pid.i <= 0.01):
        self.nidec_pid.i += 0.01  # clear out nidec pid integral while acc not controlling car

      brake, _creep_impact = compute_gb_honda_nidec(adjust_accel, out.vEgo, self.creep_factor)
      if (actuators.longControlState == LongCtrlState.pid) and (not out.stockAeb) and (not out.gasPressed) \
         and (1e-5 <= out.vEgo <= out.cruiseState.speed - 2.):
        self.creep_factor = 0.0
    else:
      self.accel = 0.0
      adjust_accel = self.accel
      brake = 0.0
    if out.gasPressed or not CC.longActive:
      self.nidec_pid.reset()

    # *** apply brake hysteresis, rate limit after the enable check ***
    pre_limit_brake, self.braking, self.brake_steady = actuator_hysteresis(brake, self.braking, self.brake_steady)
    self.brake_last = rate_limit(pre_limit_brake, self.brake_last, -2., 3 * DT_CTRL)

    wind_brake_ms2 = float(np.interp(out.vEgo, [0.0, 13.4, 22.4, 31.3, 40.2], [0.000, 0.049, 0.136, 0.267, 0.441]))  # m/s2

    # launch governor state machine: window from engaged standstill with a positive plan until moving (>1.5 m/s) or
    # timeout. Measurements per tick, learner updates once at a clean window exit
    if not self.launch_active:
      if CC.longActive and (not out.gasPressed) and (not out.brakePressed) and \
         (out.vEgo < 0.1) and (actuators.accel > 0.05):
        self.launch_active = True
        self.launch_ticks = 0
        self.launch_release_tick = -1
        self.launch_move_tick = -1
        self.launch_lurch_sum = 0.0
        self.launch_lurch_n = 0
        self.launch_err_sum = 0.0
        self.launch_err_n = 0
        self.launch_ceiling_ticks = 0
    else:
      self.launch_ticks += 1
      if (self.launch_release_tick < 0) and (self.apply_brake_last == 0):
        self.launch_release_tick = self.launch_ticks
      if (self.launch_move_tick < 0) and (out.vEgo > 0.1):
        self.launch_move_tick = self.launch_ticks
      if (self.launch_move_tick >= 0) and (10 <= self.launch_ticks - self.launch_move_tick <= 50):
        # lurch: SUSTAINED accel beyond the plan just after breakaway (0.1-0.5 s window), not the breakaway spike
        self.launch_lurch_sum += out.aEgo - actuators.accel
        self.launch_lurch_n += 1
      if self.launch_move_tick >= 0:
        self.launch_err_sum += actuators.accel - out.aEgo
        self.launch_err_n += 1
        # ceiling binding: engine running and applied pedal near the commanded seed
        if (engine_rpm > 500) and (car_gas >= 0.8 * self.car_gas_per_pcm_gas * self.gas_launch):
          self.launch_ceiling_ticks += 1
      launch_aborted = (not CC.longActive) or out.gasPressed or out.brakePressed or (actuators.accel < -0.05)
      launch_done = (out.vEgo > 1.5) or (self.launch_ticks >= 300)
      if launch_aborted or launch_done:
        if launch_done and not launch_aborted:
          never_moved = (self.launch_move_tick < 0) and (self.launch_release_tick >= 0) and \
                        (self.launch_ticks - self.launch_release_tick > 100)
          launch_slow = never_moved or ((self.launch_move_tick >= 0) and (self.launch_release_tick >= 0) and
                                        (self.launch_move_tick - self.launch_release_tick > 60))
          if launch_slow:
            self.dv_break *= 1.05
          launch_lurch = (self.launch_lurch_sum / self.launch_lurch_n) if self.launch_lurch_n > 0 else 0.0
          if launch_lurch > 0.5:
            self.dv_break *= (1 - 0.05 * min(launch_lurch, 2.0))
          self.dv_break = float(np.clip(self.dv_break, 2.0, 12.0))
          if self.launch_err_n > 50:
            launch_err = self.launch_err_sum / self.launch_err_n
            if launch_err > 0.15:
              if self.launch_ceiling_ticks > 0.5 * self.launch_err_n:
                self.gas_launch *= 1.03
              else:
                self.dv_launch *= 1.03
            elif launch_err < -0.15:
              self.gas_launch *= 0.99
              self.dv_launch *= 0.99
            self.gas_launch = float(np.clip(self.gas_launch, 40.0, P.NIDEC_GAS_MAX))
            self.dv_launch = float(np.clip(self.dv_launch, 1.0, 8.0))
        self.launch_active = False
        if launch_done and not launch_aborted:
          self.gas_recovery_ticks = 200  # hand the general pipeline a recovery window

    # recovery window on driver-gas override release and on engage while rolling
    self.gas_recovery_ticks = max(self.gas_recovery_ticks - 1, 0)
    if CC.enabled and self.gas_pressed_prev and (not out.gasPressed):
      self.gas_recovery_ticks = 200
    self.gas_pressed_prev = out.gasPressed
    if CC.longActive and (not self.long_active_prev) and (out.vEgo > 0.1) and (not self.launch_active):
      self.gas_recovery_ticks = 200
    self.long_active_prev = CC.longActive

    max_accel = float(np.interp(out.vEgo, P.NIDEC_MAX_ACCEL_BP, P.NIDEC_MAX_ACCEL_V))
    # two-band speed channel blend: pure low band below 10 m/s, pure high band above 16 m/s
    low_w = float(np.interp(out.vEgo, [10.0, 16.0], [1.0, 0.0]))
    sf_eff = low_w * self.speedfactor_low + (1.0 - low_w) * self.speedfactor
    alpha_eff = low_w * self.speedalpha_low + (1.0 - low_w) * self.speedalpha
    gas_accel = 0.0
    if not CC.longActive:
      if CC.enabled and out.gasPressed and car_gas_available:
        # driver-gas override: mirror the applied pedal onto the wire instead of zeroing it, so the PCM's pedal
        # tracker stays wound to the true operating point through the override
        pcm_speed = float(np.clip(out.vEgo, 0.0, 100.0))
        pcm_accel = int(np.clip(car_gas / max(self.car_gas_per_pcm_gas, 1e-3), 0.0, P.NIDEC_GAS_MAX))
      else:
        pcm_speed = 0.0
        pcm_accel = 0
    else:
      if self.launch_active:
        # stock-shaped launch lead; the larger breakaway lead until first motion
        speed_lead = self.dv_launch if out.vEgo > 0.1 else self.dv_break
      else:
        speed_lead = float(sf_eff * self.accel + alpha_eff)
      pcm_speed = float(np.clip(out.vEgo + speed_lead, 0.0, 100.0))
      gas_accel = adjust_accel + wind_brake_ms2 * self.windfactor
      gf_eff = low_w * self.gasfactor_low + (1.0 - low_w) * self.gasfactor
      pcm_accel = int(np.clip((self.gas_alpha + gas_accel * gf_eff / 1.44) / max_accel, 0.0, 1.0) * P.NIDEC_GAS_MAX)
    max_speedcontrol = pcm_speed > 99.999
    prior_speedfactor = self.speedfactor
    prior_speedalpha = self.speedalpha
    prior_speedfactor_low = self.speedfactor_low
    prior_speedalpha_low = self.speedalpha_low

    # feedforward for the PCM's decaying-average gas pedal, with a rise clip (wider inside recovery windows while
    # the wire is still below the target)
    prior_accel = int(self.new_accel)
    max_increase = 7 if (self.gas_recovery_ticks > 0 and prior_accel < pcm_accel) else 2  # per 100 Hz tick
    effective_average_factor = self.average_factor if car_gas_available else 0.5
    self.new_accel = int((pcm_accel - self.prior_gas_average * (1 - effective_average_factor)) / effective_average_factor)
    self.new_accel = int(np.clip(self.new_accel, 0, min(prior_accel + max_increase, P.NIDEC_GAS_MAX)))
    if self.launch_active:
      # gas seed: send the learned launch gas immediately; full authority engine-off until first motion
      launch_seed = self.gas_launch
      if (engine_rpm < 500) and (out.vEgo <= 0.1):
        launch_seed = P.NIDEC_GAS_MAX
      self.new_accel = int(min(launch_seed, P.NIDEC_GAS_MAX))
    self.average_factor_sens = (self.new_accel - self.prior_gas_average) + (1 - effective_average_factor) * self.average_factor_sens
    self.prior_gas_average = self.prior_gas_average * (1 - effective_average_factor) + (self.new_accel * effective_average_factor)

    # ---- learners ----
    if (0 < self.new_accel < P.NIDEC_GAS_MAX) and (not out.gasPressed) and \
       (self.apply_brake_last == 0) and (not self.launch_active) and (self.gas_recovery_ticks == 0):
      gasfactor_error = (self.accel - out.aEgo)
      self.gas_alpha = float(np.clip(self.gas_alpha + 0.0001 * gasfactor_error / 4.8, -3.0, 3.0))
      gf_growth = 0.0001 * gasfactor_error * gas_accel
      self.gasfactor_low = float(np.clip(self.gasfactor_low * (1 + low_w * gf_growth), 0.1, 5.0))
      self.gasfactor = float(np.clip(self.gasfactor * (1 + (1.0 - low_w) * gf_growth), 0.1, 5.0))
    if (not out.gasPressed) and (self.apply_brake_last == 0):  # adjust speedfactor and average_factor
      speedfactor_error = (self.accel - out.aEgo)
      dv_sent = sf_eff * self.accel + alpha_eff
      dv_sat = max(0.1, sf_eff * self.sat_accel + alpha_eff)

      # average_factor learner: system ID of the PCM's one-pole smoothing against the applied pedal (CAR_GAS),
      # engine running only, outside launch/recovery windows
      if CC.longActive and (out.vEgo > 1e-5) and car_gas_available and \
         (engine_rpm > 500) and (not self.launch_active) and (self.gas_recovery_ticks == 0):
        wire_gas = float(self.new_accel)
        if (wire_gas > 20.0) and (car_gas > 5.0) and (abs(wire_gas - prior_accel) <= 1.0):
          scale_sample = car_gas / wire_gas
          self.car_gas_per_pcm_gas += 0.0005 * (scale_sample - self.car_gas_per_pcm_gas)
        self.car_gas_per_pcm_gas = max(0.00001, self.car_gas_per_pcm_gas)
        gas_measured = car_gas / self.car_gas_per_pcm_gas
        averagefactor_error = (gas_measured - self.prior_gas_average) / P.NIDEC_GAS_MAX
        averagefactor_step = 0.005 * averagefactor_error * self.average_factor_sens / P.NIDEC_GAS_MAX
        self.average_factor = float(np.clip(self.average_factor + np.clip(averagefactor_step, -0.0001, 0.0001), 0.02, 1.0))

      # ceiling learner: identifies max accel capability, after a second of consistent evidence
      if (out.aEgo > self.sat_accel) and (not out.gasPressed) and CC.longActive:
        self.sat_excess_frames += 1
      else:
        self.sat_excess_frames = 0
      if (out.aEgo < self.sat_accel <= self.accel) and (not out.gasPressed) and CC.longActive and \
         (not self.launch_active) and (self.gas_recovery_ticks == 0):
        self.sat_deficit_frames += 1
      else:
        self.sat_deficit_frames = 0
      if (self.sat_excess_frames > 100) or (self.sat_deficit_frames > 100):
        self.sat_accel = float(np.clip(self.sat_accel + 0.002 * (out.aEgo - self.sat_accel), 0.1, P.NIDEC_ACCEL_MAX - 0.1))

      # speed-channel learner (outside launch/recovery windows)
      if CC.longActive and (out.vEgo > 1e-5) and (not self.launch_active) and (self.gas_recovery_ticks == 0):
        if (speedfactor_error > 0) and (dv_sent > dv_sat):
          # beyond the knee surplus dv does nothing: bleed toward the knee instead of growing
          sf_growth = -min(0.0005 * (dv_sent - dv_sat), 0.005)
        else:
          sf_growth = 0.001 * speedfactor_error * self.accel
        self.speedfactor_low = float(np.clip(self.speedfactor_low * (1 + low_w * sf_growth), 0.01, 99.0))
        self.speedfactor = float(np.clip(self.speedfactor * (1 + (1.0 - low_w) * sf_growth), 0.01, 99.0))
        self.speedalpha_low = min(dv_sat, self.speedalpha_low + low_w * 0.001 * speedfactor_error)
        self.speedalpha = min(dv_sat, self.speedalpha + (1.0 - low_w) * 0.001 * speedfactor_error)

      if max_speedcontrol or (dv_sent > dv_sat):  # only allow learning reductions
        self.speedfactor = min(prior_speedfactor, self.speedfactor)
        self.speedfactor_low = min(prior_speedfactor_low, self.speedfactor_low)
        self.speedalpha = min(prior_speedalpha, self.speedalpha)
        self.speedalpha_low = min(prior_speedalpha_low, self.speedalpha_low)
        self.windfactor = min(prior_windfactor, self.windfactor)

    # ---- brake (sent every other frame) ----
    if frame % 2 == 0:
      ts = frame * DT_CTRL
      apply_brake = float(np.clip(self.brake_last - wind_brake, 0.0, 1.0))
      if (apply_brake > 0) and (actuators.longControlState == LongCtrlState.pid) and (out.vEgo > 1e-5) and (not out.stockAeb):
        if not ((self.accel >= 1e-5) and out.vEgo < 1.0):  # don't wind PID at launch lurch
          self.brake_pid_factor = float(self.brake_pid.update(error=-(self.accel - out.aEgo) * apply_brake, speed=out.vEgo))
      if out.vEgo >= 2:  # save pid above 2 m/s
        self.brake_pid_factor_non_lowspeed = self.brake_pid_factor
      if (out.vEgo < 1e-5) and (self.accel < 1e-5):  # gradually restore 2 m/s pid after stopped
        self.brake_pid.i = float(np.clip(self.brake_pid_factor_non_lowspeed, self.brake_pid.i - 0.01, self.brake_pid.i + 0.01))
      brakefactor = 1 + self.brake_pid_factor
      apply_brake = int(np.clip(apply_brake * brakefactor * P.NIDEC_BRAKE_MAX, 0, P.NIDEC_BRAKE_MAX - 1))
      self.pump_on, self.last_pump_ts = brake_pump_hysteresis(apply_brake, self.apply_brake_last, self.last_pump_ts, ts)

      # limit brake release to 32 units per frame to match factory
      apply_brake = max(self.apply_brake_last - 32, apply_brake)

      if apply_brake > 0:  # prevent fault from concurrent gas + brake
        pcm_speed = 0.0
        self.new_accel = 0
      if CS.out.gasPressed and not car_gas_available:
        self.new_accel = 0

      # openpilot brake release: the gas channel restarts from a dead state, open a recovery window
      if (self.apply_brake_last > 0) and (apply_brake == 0) and CC.longActive and (not self.launch_active):
        self.gas_recovery_ticks = 200

      if out.vEgo < out.cruiseState.speed - 2.:
        self.gasfactor_nomaxspeed = self.gasfactor
        self.gasfactor_low_nomaxspeed = self.gasfactor_low
        self.gas_alpha_nomaxspeed = self.gas_alpha
      else:
        self.gasfactor_nomaxspeed = min(self.gasfactor_nomaxspeed, self.gasfactor)
        self.gasfactor_low_nomaxspeed = min(self.gasfactor_low_nomaxspeed, self.gasfactor_low)
        self.gas_alpha_nomaxspeed = min(self.gas_alpha_nomaxspeed, self.gas_alpha)

      self.apply_brake_last = apply_brake

    if frame % SAVE_INTERVAL_FRAMES == 0 and frame > 0:
      record = save_state(self.store, self.learned_state())
      carlog.warning(json.dumps({"event": "honda_nidec_long_saved", **record}))

    return AdaptiveOutput(
      pcm_speed=pcm_speed,
      pcm_gas=int(self.new_accel),
      set_me_x01=1 if (self.launch_active or self.new_accel == 0 or self.new_accel == P.NIDEC_GAS_MAX) else 0,
      apply_brake=int(self.apply_brake_last),
      pump_on=self.pump_on,
      accel=float(self.accel),
      gas_target=pcm_accel / P.NIDEC_GAS_MAX,
    )
