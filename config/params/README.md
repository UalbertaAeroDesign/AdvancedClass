# Flight controller parameter files

ArduPilot parameter snapshots for each airframe / flight phase. Load into
Mission Planner via **Config → Full Parameter List → Load from file**.

Naming convention: `YEAR_descriptor_versionN.params`

| File | Airframe | Phase | Notes |
|---|---|---|---|
| `2026_precomp_v1.params` | Tricopter VTOL | Pre-competition | First baseline with `Q_TILT_*` tuning before yaw fix |

When making changes, **save a new file** rather than overwriting an existing
one — keeps the rollback path clean. Commit the file with a message describing
what changed and why (e.g. "bump Q_TILT_YAW_ANGLE to 30 for yaw authority").
