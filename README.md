# SMART3D-MultiAgent Private Deployment Runbook

This private repository is the real deployment runbook. It contains i9 / Jetson
/ Pi / Wheeltec connection details, startup order, logs, backups, and
hardware-specific recovery notes. Current Jetson default is LiDAR + IMU +
Cartographer without wheel odom fused by default.

Do not copy this repository into the LinksLab/public release. The public release
should say:

> This repository reproduces SMART-3D / PECMAN simulations, benchmarks, and clean
> paper RViz captures. Hardware deployment files, hostnames, IPs, access methods,
> and field recovery logs are intentionally excluded.

## Current Truth

- Public paper/simulation code lives in the LinksLab-style SMART-3D repository.
- This repository is the private field notebook for real robots, networking,
  startup order, recovery logs, and hardware-specific scripts.
- The accepted 2026-05-08 paper visual lane is:
  `shared unknown + king + cross diagonal goals + no pedestrians + detour=1.5 +
  clean paper RViz`.
- The current Jetson robot baseline is intentionally conservative:
  LiDAR + IMU + `scan_metadata_normalizer.py` + Cartographer, with wheel odom not
  fused into Cartographer by default.
- Wheels still execute motion commands in active mode. They are not currently
  trusted as the default SLAM fusion source.

## Machine Roles

| Role | Purpose | Notes |
|------|---------|-------|
| Central desktop | Runs planner, RViz, validation, and multi-robot coordination | Older notes may call it `i9`; public docs should call it the central computer or central desktop. |
| Jetson Orin NX 16GB robot | Current main field robot runtime | Default baseline is no-wheel-odom fusion. |
| Raspberry Pi 5 robot | Original reference chain and historical deployment baseline | Pi5 formal target was LiDAR + IMU + wheel odom into Cartographer. |
| Wheeltec Raspberry Pi 5 robot | Separate two-wheel ROS1/ROS2 migration target | Treat as a separate port until its runbook is fully closed. |
| MacBook | Editing, deployment orchestration, and documentation | Not the runtime benchmark source of truth. |

## Start Here

Use these files first when resuming field work:

- `SMART3D_CAMPUS_CONNECTION_GUIDE.md`
- `SMART3D_CAMPUS_RUNTIME.md`
- `SMART3D_NEXT_SESSION_HANDOFF_2026-04-18.md`
- `SMART3D_JETSON_CAMPUS_RUNTIME_2026-04-18.md`
- `SMART3D_JETSON_PI5_REFERENCE_RESTORE_2026-04-19.md`
- `scripts/smart3d_campus.sh`
- `scripts/smart3d_jetson_campus.sh`
- `jetson/jetson_runtime_ctl.sh`
- `jetson/deploy_runtime_to_jetson.sh`

Many old files are historical snapshots. When two docs disagree, prefer this
README plus the newest Jetson restore/runtime notes.

## Jetson Default Baseline

The important defaults are in `scripts/smart3d_jetson_campus.sh` and
`jetson/start_pi5_reference_passive.sh`:

```bash
START_PI5_REFERENCE_WHEEL_ODOM=0
JETSON_PI5_REFERENCE_CARTO_CONFIG_BASENAME=robotcar_2d.lua
SMART3D_CARTO_CONFIG_BASENAME=robotcar_2d.lua
```

This means:

- LiDAR publishes `/scan`.
- `scan_metadata_normalizer.py` republishes the SLAM-facing scan, normally
  `/scan_slam`.
- HiPNUC IMU publishes `/IMU_data`.
- Cartographer runs the no-wheel-odom config by default.
- Wheel odom is an explicit experiment, not the default baseline.

Do not describe this as "original Pi5 fully restored." It is a Jetson-stable
baseline that preserves the Pi5-style responsibility split while avoiding bad
wheel odom as a default SLAM truth source.

## Jetson Typical Flow

From the Mac/private checkout:

```bash
scripts/smart3d_jetson_campus.sh prepare
scripts/smart3d_jetson_campus.sh start-passive
scripts/smart3d_jetson_campus.sh validate
scripts/smart3d_jetson_campus.sh start-active
scripts/smart3d_jetson_campus.sh goal-test
```

Stop cleanly:

```bash
scripts/smart3d_jetson_campus.sh stop
```

Use `START_PI5_REFERENCE_WHEEL_ODOM=1` only for a deliberate wheel-odom fusion
test after RS485/common-ground/feedback quality is verified.

## Pi5 Reference Meaning

The original Pi5 formal chain was:

```text
/scan + /IMU_data + /odom -> Cartographer robotcar_2d_odom.lua
```

The split was:

- passive: static TF, LiDAR, IMU, IMU bridge, `ddsm115_odom.py`, Cartographer
- active: `cmd_vel_driver.py` plus `pi5_client.py`

The Jetson work intentionally borrowed this split, but the current stable
Jetson baseline does not fuse wheel odom by default.

## Perfect Backups

The `backups/` directory contains named working snapshots. In particular:

- `perfect2` means planner + follower + RViz display were accepted.
- It does not mean full original Pi5 three-source wheel-odom fusion was restored.
- In perfect2, wheels execute motion; LiDAR + IMU are the default localization
  baseline.

Always read the backup README before copying files out of a snapshot.

## Known Pitfalls

- Do not mix passive SLAM, active driving, keyboard direct control, wheel odom,
  and brake guards without first deciding who owns the motor serial port.
- Do not assume wheel odom is fixed just because the robot can drive.
- Do not run camera streaming during SLAM stability tests unless the test is
  explicitly about camera load; video can amplify Jetson timing problems.
- Do not treat `Dropped earlier points` or Cartographer subdivision warnings as
  harmless if the robot suddenly jumps in RViz.
- Do not collapse the Pi5 history into "LiDAR only." The original Pi5 formal
  target used LiDAR, IMU, and wheel odom.
- Do not copy real access details, private IPs, hostnames, logs, or recovery
  notes into the LinksLab/public repository.

## Wheeltec Status

The Wheeltec Pi5 is a separate hardware port with a ROS1 history and an STM32
wheel-control layer. It should not be silently treated as the Jetson robot.
Before making it part of the multi-agent runtime, verify:

- LiDAR topic and frame naming
- base frame and TF chain
- `/cmd_vel` behavior
- wheel-control latency
- ROS1/ROS2 bridge or full ROS2 port choice
- central planner namespace and RViz display separation from the Jetson robot

## Public vs Private Wording

For LinksLab/public README:

```text
This repository reproduces SMART-3D / PECMAN simulations, benchmarks, and clean
paper RViz captures. Hardware deployment files, hostnames, IPs, access methods,
and field recovery logs are intentionally excluded.
```

For this private README:

```text
This private repository is the real deployment runbook. It contains i9 / Jetson
/ Pi / Wheeltec connection details, startup order, logs, backups, and
hardware-specific recovery notes. Current Jetson default is LiDAR + IMU +
Cartographer without wheel odom fused by default.
```
