"""
Persistence for learned controller values, kept out of opendbc: the openpilot side injects a store that writes one
JSON param ("HondaNidecLongState"); tests and plain opendbc use MemoryStore.

The state is versioned: a different STATE_VERSION (or anything unreadable) loads defaults, so a changed controller
never starts from incompatible learned values. Every value is clamped to its bounds on load.
"""
import json
import math
import os

STATE_VERSION = 1

# Learned values committed to the repo, used when the device has no stored state (fresh install, new device, factory
# reset). Create/refresh it from a device with tools/honda_nidec_long_state.py export (sunnypilot repo).
SEED_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "learned_seed.json")


def load_seed(path: str = SEED_FILE) -> dict | None:
  try:
    with open(path, encoding="utf-8") as f:
      return json.load(f)
  except (OSError, ValueError):
    return None


class MemoryStore:
  """Store interface: load() -> dict | None, save(dict) -> None. save() must not block the caller."""
  def __init__(self, data: dict | None = None):
    self.data = data

  def load(self) -> dict | None:
    return self.data

  def save(self, data: dict) -> None:
    self.data = dict(data)


def _valid(raw) -> bool:
  return isinstance(raw, dict) and raw.get("version") == STATE_VERSION and isinstance(raw.get("values"), dict)


def load_state(store, defaults: dict, bounds: dict, seed: dict | None = None) -> tuple[dict, str]:
  """Returns (state, source) with source one of "stored", "seed", "defaults"."""
  state = dict(defaults)
  try:
    raw = store.load() if store is not None else None
  except Exception:
    raw = None
  source = "stored"
  if not _valid(raw):
    raw, source = seed, "seed"
  if not _valid(raw):
    return state, "defaults"
  values = raw.get("values", {})
  for key, default in defaults.items():
    value = values.get(key, default)
    try:
      value = float(value)
    except (TypeError, ValueError):
      value = default
    if not math.isfinite(value):
      value = default
    lo, hi = bounds.get(key, (-math.inf, math.inf))
    state[key] = min(max(value, lo), hi)
  return state, source


def make_record(state: dict) -> dict:
  return {"version": STATE_VERSION, "values": {k: round(float(v), 6) for k, v in state.items()}}


def save_state(store, state: dict) -> dict:
  record = make_record(state)
  if store is not None:
    try:
      store.save(record)
    except Exception:
      pass
  return record
