# adaptive.py provenance

Ported from the MVL fork (mvl-boston/openpilot), `opendbc_repo/opendbc/car/honda/carcontroller.py` at opendbc commit
`3278c6bef` ("Honda Nidec: low-speed gasfactor band, gas learners out of the speed-channel ratchet"), 2026-11-15.

| MVL carcontroller.py (3278c6bef) | adaptive.py |
|---|---|
| `compute_gb_honda_nidec` (creep factor), `actuator_hysteresis`, `brake_pump_hysteresis` | module functions of the same names |
| `__init__` learned values read from `Honda*Params` (lines ~147-245) | `LEARNED` table + `load_state()` (one JSON record) |
| `nidec_pid` / `brake_pid` construction (~155-169) | `__init__` |
| update: pitch / hill brake, wind brake, nidec_pid, windfactor/gasfactor ratchets (270-332) | `update()` first block |
| brake hysteresis + `rate_limit(..., 3 * DT_CTRL)` (343-348) | `update()` |
| launch governor (383-460) | `update()` launch block |
| gas recovery windows (461-473) | `update()` |
| PCM speed lead / gas target, driver-gas mirror (476-512) | `update()` |
| PCM smoothing feedforward, rise clip, launch seed (513-554) | `update()` |
| learners: gas, average_factor, car_gas scale, sat_accel, speed bands (556-673) | `update()` learners block |
| Nidec brake branch: brake PID, release limit, gas/brake guard, nomaxspeed (741-795) | `update()` brake block (even frames) |
| `create_acc_hud(..., launch)` SET_ME_X01 rule | `AdaptiveOutput.set_me_x01` -> `hondacan.create_acc_hud(set_me_x01=)` |
| carstate `car_gas`, `car_gas_available`, `engine_rpm` | `CarStateExt` (GAS_PEDAL 0x13C only, the MDX has no 0x130) |
| interface Nidec `kiV` disabled | `apply_mode(ADAPTIVE)`: kiV = [0], delay 0.15 |
| `HondaParamWriter` + 16 `Honda*Params` | injected store; openpilot `HondaNidecLongState` (JSON) |

Not ported: Bosch paths, lateral `latFactors` learner, MVL's reuse of actuator output fields for debugging
(`speed=nidec_pid_factor`, `gas=average_factor`, ...); learned values are logged with carlog instead.
Kept from sunnypilot instead of MVL: `pcm_override = True` in BRAKE_COMMAND, NIDEC_ACCEL_MAX 1.6 (MVL 2.0).

## Re-sync with a newer MVL
1. `git -C <MVL>/opendbc_repo diff 3278c6bef <new> -- opendbc/car/honda/carcontroller.py opendbc/car/honda/carstate.py`
2. Apply the Nidec logic changes to the matching block of adaptive.py (the table above). New learned values:
   add to `LEARNED` with MVL's default and clip; a changed meaning of an existing one: bump `STATE_VERSION` in store.py.
3. Run `pytest opendbc/sunnypilot/car/honda/nidec_long` and the log replay, then update the commit above.
