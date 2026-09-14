# Door-topology v1 archive

This directory contains the complete pre-coverage Stage B door state machine,
its launch files, offline harness, and unit tests.  It is intentionally outside
the active `scripts/`, `launch/`, and `test/` directories, so the default build
and launch cannot start it accidentally.

The active implementation is `scripts/coverage_explorer_node.py`.  Restore
files from this archive only when deliberately reverting the strategy; do not
mix the two explorers because both publish `/cmd_vel`.
