## Jetson No-Odom Gold Backup

Backup date: `2026-04-19` America/New_York

### What this is

This backup freezes the **pre-wheel-odom** Jetson runtime that the user reported felt more stable than the later odom-enabled version.

It is the version to restore first when:

- long-run drift needs to be compared against the odom-enabled chain
- wheel odom introduces extra instability
- the goal is to recover the earlier `LiDAR + IMU + Smart3D` baseline before re-testing `/odom`

### Core runtime shape

This backup should be used with:

- Cartographer config: `robotcar_2d.lua`
- LiDAR input to Cartographer: `/scan_slam`
- `scan_metadata_normalizer.py` enabled
- IMU enabled
- wheel odom **not fused**
- `START_PI5_REFERENCE_WHEEL_ODOM=0`

### Files included

- `start_pi5_reference_passive.sh`
- `start_pi5_reference_active.sh`
- `start_pi5_reference_campus_support.sh`
- `jetson_runtime_ctl.sh`
- `deploy_runtime_to_jetson.sh`
- `scan_metadata_normalizer.py`
- `m10p_uart.yaml`
- `smart3d_jetson_campus.sh`
- `pi5_client.py`
- `cartographer_launch.py`
- `robotcar_2d.lua`
- `i9_planner.py`

### Restore intent

If you want to go back to this gold version, the main runtime knobs should be:

- `JETSON_RUNTIME_PROFILE=pi5_reference`
- `JETSON_PI5_REFERENCE_CARTO_CONFIG_BASENAME=robotcar_2d.lua`
- `START_PI5_REFERENCE_WHEEL_ODOM=0`
- `KEEP_PI5_REFERENCE_WHEEL_ODOM=0`

### Canonical restart

```bash
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/deploy_runtime_to_jetson.sh

JETSON_RUNTIME_PROFILE=pi5_reference \
JETSON_PI5_REFERENCE_CARTO_CONFIG_BASENAME=robotcar_2d.lua \
START_PI5_REFERENCE_WHEEL_ODOM=0 \
KEEP_PI5_REFERENCE_WHEEL_ODOM=0 \
PI5_EMERGENCY_STOP_DIST=0.05 \
ROBOT_SPEED=0.24 \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh start-passive

JETSON_RUNTIME_PROFILE=pi5_reference \
JETSON_PI5_REFERENCE_CARTO_CONFIG_BASENAME=robotcar_2d.lua \
START_PI5_REFERENCE_WHEEL_ODOM=0 \
KEEP_PI5_REFERENCE_WHEEL_ODOM=0 \
PI5_EMERGENCY_STOP_DIST=0.05 \
ROBOT_SPEED=0.24 \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_jetson_campus.sh start-active
```

### Why this backup matters

This is the best "clean comparison point" against the later version that reintroduced `/odom` using `robotcar_2d_odom.lua`.

Use this backup when debugging:

- sudden long-run drift
- `Dropped earlier points`
- whether wheel odom helps or hurts
- whether the system was better before `/odom` came back
