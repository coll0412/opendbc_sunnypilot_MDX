import math
import unittest
from unittest import mock

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR
from opendbc.sunnypilot.car.honda.fingerprints_ext import FINGERPRINTS_EXT
from opendbc.sunnypilot.car.honda.values_ext import HondaFlagsSP
from opendbc.sunnypilot.car.honda.nidec_long import NidecLongMode, apply_mode, get_mode
from opendbc.sunnypilot.car.honda.nidec_long import adaptive as adaptive_mod
from opendbc.sunnypilot.car.honda.nidec_long.store import MemoryStore, STATE_VERSION, load_state, make_record

MDX = CAR.ACURA_MDX_3G_SERIAL


def get_params(car=MDX):
  fp = gen_empty_fingerprint()
  if car == MDX:
    fp[0] = dict(FINGERPRINTS_EXT[MDX][0])
  CI = interfaces[car]
  CP = CI.get_params(car, fp, [], False, False, False)
  CP_SP = CI.get_params_sp(CP, car, fp, [], False, False, False)
  return CP, CP_SP


def drive(CP, CP_SP, store=None, frames=200, accel=0.5):
  CI = interfaces[CP.carFingerprint](CP, CP_SP)
  CI.CC.nidec_long_store = store
  sends = []
  for i in range(frames):
    CI.update([(i * 10_000_000, [])])
    CC = structs.CarControl()
    CC.enabled = CC.longActive = True
    CC.actuators.accel = accel
    CC.actuators.longControlState = structs.CarControl.Actuators.LongControlState.pid
    _, s = CI.apply(CC.as_reader(), structs.CarControlSP(), i * 10_000_000)
    sends += s
  return CI, sends


class TestNidecLongModes(unittest.TestCase):
  def test_default_is_mapped(self):
    CP, CP_SP = get_params()
    self.assertEqual(get_mode(CP, CP_SP), NidecLongMode.MAPPED)

  def test_apply_mode(self):
    for mode, flag, ki, delay in ((NidecLongMode.STOCK, HondaFlagsSP.NIDEC_LONG_STOCK, 1.2, 0.15),
                                  (NidecLongMode.MAPPED, 0, 0.6, 0.5),
                                  (NidecLongMode.ADAPTIVE, HondaFlagsSP.NIDEC_LONG_ADAPTIVE, 0.0, 0.15)):
      CP, CP_SP = get_params()
      self.assertEqual(apply_mode(CP, CP_SP, int(mode)), mode)
      self.assertEqual(get_mode(CP, CP_SP), mode)
      if flag:
        self.assertTrue(CP_SP.flags & flag)
      self.assertAlmostEqual(CP.longitudinalTuning.kiV[0], ki, places=4)
      self.assertAlmostEqual(CP.longitudinalActuatorDelay, delay, places=4)

  def test_switching_back_clears_flags(self):
    CP, CP_SP = get_params()
    apply_mode(CP, CP_SP, 2)
    apply_mode(CP, CP_SP, 1)
    self.assertEqual(get_mode(CP, CP_SP), NidecLongMode.MAPPED)
    self.assertFalse(CP_SP.flags & (HondaFlagsSP.NIDEC_LONG_STOCK | HondaFlagsSP.NIDEC_LONG_ADAPTIVE))

  def test_invalid_mode_is_mapped(self):
    CP, CP_SP = get_params()
    self.assertEqual(apply_mode(CP, CP_SP, 7), NidecLongMode.MAPPED)
    self.assertEqual(apply_mode(CP, CP_SP, "x"), NidecLongMode.MAPPED)

  def test_mode_from_sunnypilot_params_list(self):
    # the device path: openpilot's initialize_params list -> opendbc sunnypilot setup_interfaces
    from opendbc.sunnypilot.car.interfaces import setup_interfaces
    for value, expected in ((2, NidecLongMode.ADAPTIVE), (0, NidecLongMode.STOCK), (1, NidecLongMode.MAPPED)):
      CP, CP_SP = get_params()
      setup_interfaces(interfaces[MDX], CP, CP_SP, [{"HondaNidecLongMode": value}])
      self.assertEqual(get_mode(CP, CP_SP), expected)
    CP, CP_SP = get_params()
    setup_interfaces(interfaces[MDX], CP, CP_SP, [])
    self.assertEqual(get_mode(CP, CP_SP), NidecLongMode.MAPPED)

  def test_other_cars_untouched(self):
    CP, CP_SP = get_params(CAR.HONDA_PILOT)
    ki, flags = list(CP.longitudinalTuning.kiV), CP_SP.flags
    self.assertEqual(apply_mode(CP, CP_SP, 2), NidecLongMode.STOCK)
    self.assertEqual(list(CP.longitudinalTuning.kiV), ki)
    self.assertEqual(CP_SP.flags, flags)


class TestStore(unittest.TestCase):
  defaults = {"a": 1.0, "b": 2.0}
  bounds = {"a": (0.0, 5.0), "b": (0.0, 3.0)}

  def test_stored_then_seed_then_defaults(self):
    stored = make_record({"a": 4.0, "b": 1.0})
    seed = make_record({"a": 3.0, "b": 0.5})
    self.assertEqual(load_state(MemoryStore(stored), self.defaults, self.bounds, seed), ({"a": 4.0, "b": 1.0}, "stored"))
    self.assertEqual(load_state(MemoryStore(None), self.defaults, self.bounds, seed), ({"a": 3.0, "b": 0.5}, "seed"))
    self.assertEqual(load_state(MemoryStore(None), self.defaults, self.bounds, None), (self.defaults, "defaults"))

  def test_version_mismatch_ignored(self):
    old = {"version": STATE_VERSION + 1, "values": {"a": 4.0}}
    self.assertEqual(load_state(MemoryStore(old), self.defaults, self.bounds)[1], "defaults")

  def test_clamp_and_bad_values(self):
    raw = {"version": STATE_VERSION, "values": {"a": 99.0, "b": math.nan}}
    state, _ = load_state(MemoryStore(raw), self.defaults, self.bounds)
    self.assertEqual(state, {"a": 5.0, "b": 2.0})


class TestAdaptive(unittest.TestCase):
  def test_adaptive_runs_and_saves(self):
    CP, CP_SP = get_params()
    apply_mode(CP, CP_SP, 2)
    store = MemoryStore()
    with mock.patch.object(adaptive_mod, "SAVE_INTERVAL_FRAMES", 100):
      CI, sends = drive(CP, CP_SP, store)
    self.assertIsNotNone(CI.CC.nidec_adaptive)
    self.assertFalse(CI.CC.nidec_adaptive_failed)
    self.assertEqual(store.data["version"], STATE_VERSION)
    self.assertEqual(set(store.data["values"]), set(adaptive_mod.LEARNED))
    self.assertTrue(any(a == 0x30C for a, _, _ in sends))  # ACC_HUD still sent

  def test_adaptive_resumes_from_store(self):
    CP, CP_SP = get_params()
    apply_mode(CP, CP_SP, 2)
    store = MemoryStore(make_record({"speedfactor": 2.5, "gasfactor": 1.7}))
    CI, _ = drive(CP, CP_SP, store, frames=2, accel=0.0)
    self.assertAlmostEqual(CI.CC.nidec_adaptive.speedfactor, 2.5, places=4)
    self.assertAlmostEqual(CI.CC.nidec_adaptive.gasfactor, 1.7, places=4)

  def test_failure_falls_back_to_mapped(self):
    CP, CP_SP = get_params()
    apply_mode(CP, CP_SP, 2)
    with mock.patch.object(adaptive_mod.NidecAdaptiveLongitudinal, "update", side_effect=RuntimeError("boom")):
      CI, sends = drive(CP, CP_SP, frames=20)
    self.assertTrue(CI.CC.nidec_adaptive_failed)
    self.assertTrue(any(a == 0xE4 for a, _, _ in sends))  # car controller kept running


if __name__ == "__main__":
  unittest.main()
