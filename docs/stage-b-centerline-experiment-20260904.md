# Stage B corridor-centering candidate (2026-09-04)

Status: rejected and rolled back. The original frozen single-floor baseline is
restored and `config/stage_b_floor0_frozen.sha256` passes for all three files.

## Baseline evidence

The unchanged single-floor baseline was protected by
`config/stage_b_floor0_frozen.sha256`. Five same-seed door-lock trials showed:

- Success at arrival cross-track errors 0.078 m, 0.038 m, and 0.021 m.
- Failure at arrival cross-track errors 0.394 m and 0.278 m.
- Failed straight rescans increased cross-track error to 0.626 m and 0.392 m.

## Candidate changes

Only pre-room corridor motion is changed:

- `coverage_explorer_core.py`: bounded centerline heading and guarded wall
  clearance error helpers.
- `coverage_explorer_node.py`: apply centerline feedback during the last 4 m
  of lobby transit and during bounded door rescans.
- `stage_b_behavior.launch`: gains 0.45 (coordinate fallback), 0.70 (valid
  wall pair), 0.20 rad maximum correction, and 0.03 m deadband.
- `test_corridor_centering.py`: correction sign, clamp, and open-door rejection.

Room portal geometry, temporal confirmation, room scheduling, A* exploration,
room entry/exit, coverage thresholds, and motion speeds are unchanged.

Wall centering is accepted only when both side ranges are finite and their sum
is between 0.65 and 1.45 times the configured 2.2 m corridor width. Otherwise
the controller falls back to the entrance-derived coordinate centerline. This
prevents an open room door from pulling the robot sideways.

## Results so far

- Coordinate-only candidate trial 1: success; arrival 40.486 s, cross-track
  0.064 m, door locked 46.712 s.
- Coordinate-only candidate trial 2: failure; localization cross-track stayed
  within 0.028-0.078 m, but Gazebo physical x was 0.486 m while localization x
  was -0.097 m (about 0.58 m slow lateral localization error). This motivated
  guarded wall-range feedback; the coordinate-only version must not be frozen.
- Guarded wall-range trial 1: failure. Wall feedback improved the physical x
  position from -0.631 m to about -0.093 m and physical heading to 89.4 degrees,
  while localization drifted to x=+0.623 m. Despite the improved physical
  trajectory, the scan completed all 3 m with no portal evidence by 68.3 s.
  The occupancy-map doorway geometry had already been distorted by localization
  drift, so motion correction alone could not recover door recognition.

## Acceptance and rollback

Accept only after repeated headless real Gazebo trials lock the first door
without materially slowing the roughly 40 s corridor arrival, followed by a
successful complete single-floor run. Then regenerate the frozen checksum.

The candidate was removed after the failed guarded wall-range trial. No portal
detector, room scheduler, A* explorer, room entry/exit, coverage threshold, or
motion-speed code was changed by the experiment. The existing frozen checksum
was used as the authoritative rollback test and passes exactly.
