# Wheel + IMU/LiDAR reference backup

- Purpose: preserve the wheel-prior baseline and the current repaired LiDAR+IMU timing chain.
- Notes: `robotcar_2d_odom.lua` is the wheel-odom Cartographer config; `robotcar_2d.lua` is no-wheel-odom.
- Timing fix snapshot: `scan_metadata_normalizer.py` here includes strict monotonic retiming/drop guards.
