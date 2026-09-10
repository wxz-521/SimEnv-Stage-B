# Stage B segmented door-search countersteer (2026-09-04)

Status: frozen after the five-seed short candidate test passed 5/5. A complete
single-floor regression is still required before claiming full-floor stability,
but multi-floor work must not modify these frozen files.

The pre-experiment baseline is commit `808e8b33455dbe25434b4607aa3b9ff26c033d26`
and `config/stage_b_floor0_frozen.sha256`.

## Only behavior change

The existing one-metre advance, stop, and rescan sequence is unchanged. During
each advance, its old left/right clearance correction is combined with a small
countersteer derived from the current lateral coordinate relative to the
entrance-defined corridor centreline:

- right of centre: turn slightly left;
- left of centre: turn slightly right;
- lateral gain: 0.18 rad/m;
- lateral contribution cap: 0.08 rad;
- original left/right clearance gain remains 0.10;
- combined correction cap: 0.12 rad (the original maximum).

There is no separate turn, hold, return-to-yaw state, or extra forward segment.
Initial lobby transit, portal recognition, temporal confirmation, room logic,
A*, coverage, and speeds are unchanged.

## Short-test acceptance

Run seeds 20260901 through 20260905 headless without RViz. Stop each test as
soon as a portal reaches 3/3 evidence, or after the existing 3 m search limit
is exhausted. Record corridor-arrival pose, first evidence time, lock time,
and final search distance.

## Result

All five headless real Gazebo short tests locked `ROOM_R_15` at 3/3 evidence:

| seed | first evidence | locked | search travel |
|---|---:|---:|---:|
| 20260901 | 46.308 s | 48.210 s | 0.000 m |
| 20260902 | 42.912 s | 44.914 s | 0.000 m |
| 20260903 | 46.908 s | 48.908 s | 1.022 m |
| 20260904 | 46.910 s | 48.910 s | 1.023 m |
| 20260905 | 46.612 s | 48.612 s | 1.018 m |

The final three seeds had no portal at corridor arrival and obtained the first
candidate immediately after the original one-metre segmented search with the
new bounded countersteer. Logs are under
`logs/door_countersteer_5seed_20260904/`.

## Rollback

Remove `corridor_countersteer_heading`, its node import and two parameters,
replace its `_control_door_search` call with the frozen left/right correction,
remove the two launch parameters and `test_door_search_countersteer.py`, then
require all frozen checksum entries to pass exactly.
