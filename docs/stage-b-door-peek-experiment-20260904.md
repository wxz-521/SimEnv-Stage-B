# Stage B door-peek candidate (2026-09-04)

Status: experimental. The existing single-floor frozen checksum remains the
rollback baseline and must not be regenerated until this candidate passes.

## Change boundary

After the existing one-metre door-search segment stops with no portal:

1. Accept a side-clearance pair only when it resembles the configured 2.2 m
   corridor rather than an open room door.
2. If nearer the right wall, rotate left in place to 0.35 rad; if nearer the
   left wall, rotate right. This creates a roughly 20-degree opposing-door
   view before any further longitudinal movement.
3. Hold for 1.2 simulation seconds so lidar can update the occupancy map.
4. Restore the recorded corridor yaw in place. Only then may the existing
   one-metre search controller advance, using its original bounded left/right
   clearance heading correction. The 20-degree peek angle is never carried
   into translational motion, preventing cumulative lateral deviation.

Each one-metre observation station performs at most one peek. A centred robot
or unreliable/open-side measurement skips the peek. Portal geometry, temporal
confirmation, room scheduling, A* exploration, room entry/exit, coverage, and
all translational speeds remain unchanged.

## Rollback

If the five-seed short test regresses, remove `door_peek_turn_direction`, all
`door_peek_*` node state/control/status fields, the three launch parameters,
and `test_door_peek.py`. Then require all entries in
`config/stage_b_floor0_frozen.sha256` to pass exactly.
