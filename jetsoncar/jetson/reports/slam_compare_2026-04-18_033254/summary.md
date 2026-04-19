# Jetson SLAM Backend Compare

Output dir: `/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/reports/slam_compare_2026-04-18_033254`

| Metric | cartographer | slam_toolbox |
| --- | ---: | ---: |
| avg CPU lidar (%) | 10.72 | 10.11 |
| max RSS lidar (MB) | 43.04 | 43.07 |
| avg CPU imu (%) | 5.05 | 4.81 |
| max RSS imu (MB) | 40.79 | 40.8 |
| avg CPU cartographer (%) | 53.84 | n/a |
| max RSS cartographer (MB) | 62.78 | n/a |
| avg CPU slam_toolbox (%) | n/a | 4.4 |
| max RSS slam_toolbox (MB) | n/a | 37.46 |
| avg CPU scan_normalizer (%) | n/a | 6.95 |
| max RSS scan_normalizer (MB) | n/a | 52.85 |
| avg CPU stp23l (%) | 21.8 | 21.53 |
| max RSS stp23l (MB) | 53.23 | 53.21 |
| avg CPU stp23l_guard (%) | 3.19 | 3.26 |
| max RSS stp23l_guard (MB) | 52.21 | 56.2 |
| avg CPU map_bridge (%) | 1.15 | 1.08 |
| max RSS map_bridge (MB) | 53.26 | 49.56 |
| avg CPU pi5_client (%) | 31.9 | 17.28 |
| max RSS pi5_client (MB) | 51.68 | 53.7 |
| avg CPU watchdog (%) | 21.22 | 20.74 |
| max RSS watchdog (MB) | 53.86 | 51.62 |
| topic count scan (6s) | 72 | 72 |
| topic count imu (6s) | 601 | 602 |
| topic count front_range (6s) | 60 | 61 |
| topic count map (6s) | 10 | 7 |
| topic count map_campus (6s) | 6 | 6 |
| topic count pose (6s) | 60 | 60 |
| topic count image_raw (6s) | 180 | 180 |
| topic count camera_info (6s) | 180 | 180 |

## Notes

### cartographer
- counts_6s: `{'scan': 72, 'imu': 601, 'front_range': 60, 'map': 10, 'map_campus': 6, 'pose': 60, 'image_raw': 180, 'camera_info': 180}`
- lidar tail:
```text
[INFO] [1776497602.170525931] [lslidar_driver_node]: Initialised lslidar without error
[INFO] [1776497602.170769140] [lslidar_driver_node]: Successfully initialize driver...
port = /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6D010263-if00, baud_rate = 500000
open_port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6D010263-if00 OK !
degree_compensation = -327.670000
```
- cartographer tail:
```text
[cartographer_node-1] [WARN] [1776497703.318556774] [cartographer logger]: W0418 03:35:03.000000 58681 range_data_collator.cc:82] Dropped 3 earlier points.
[cartographer_node-1] [WARN] [1776497703.487740414] [cartographer logger]: W0418 03:35:03.000000 58681 range_data_collator.cc:82] Dropped 1 earlier points.
[cartographer_node-1] [INFO] [1776497703.609906473] [cartographer logger]: I0418 03:35:03.000000 58681 collated_trajectory_builder.cc:81] imu rate: 100.00 Hz 1.00e-02 s +/- 7.16e-05 s (pulsed at 99.94% real time)
[cartographer_node-1] [INFO] [1776497703.610019117] [cartographer logger]: I0418 03:35:03.000000 58681 collated_trajectory_builder.cc:81] scan rate: 11.93 Hz 8.38e-02 s +/- 6.19e-03 s (pulsed at 100.05% real time)
[cartographer_node-1] [WARN] [1776497703.827700707] [cartographer logger]: W0418 03:35:03.000000 58681 range_data_collator.cc:82] Dropped 1 earlier points.
```
- pi5_client tail:
```text
[INFO] [1776497689.068762716] [pi5_client]: Path received: 3 pts
[INFO] [1776497692.106203391] [pi5_client]: Path received: 3 pts
[INFO] [1776497695.086041778] [pi5_client]: Path received: 3 pts
[INFO] [1776497698.154547853] [pi5_client]: Path received: 3 pts
[INFO] [1776497701.175422576] [pi5_client]: Path received: 3 pts
```

### slam_toolbox
- counts_6s: `{'scan': 72, 'imu': 602, 'front_range': 61, 'map': 7, 'map_campus': 6, 'pose': 60, 'image_raw': 180, 'camera_info': 180}`
- lidar tail:
```text
[INFO] [1776497723.623253419] [lslidar_driver_node]: Initialised lslidar without error
[INFO] [1776497723.623365070] [lslidar_driver_node]: Successfully initialize driver...
port = /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6D010263-if00, baud_rate = 500000
open_port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6D010263-if00 OK !
degree_compensation = -327.670000
```
- slam_toolbox tail:
```text
[async_slam_toolbox_node-1] [INFO] [1776497817.124117510] [slam_toolbox]: Message Filter dropping message: frame 'laser_link' at time 1776497817.034 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-1] [INFO] [1776497819.707402084] [slam_toolbox]: Message Filter dropping message: frame 'laser_link' at time 1776497819.619 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-1] [INFO] [1776497822.288755071] [slam_toolbox]: Message Filter dropping message: frame 'laser_link' at time 1776497822.201 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-1] [INFO] [1776497824.872357496] [slam_toolbox]: Message Filter dropping message: frame 'laser_link' at time 1776497824.784 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-1] [INFO] [1776497827.375183664] [slam_toolbox]: Message Filter dropping message: frame 'laser_link' at time 1776497827.287 for reason 'discarding message because the queue is full'
```
- pi5_client tail:
```text
[INFO] [1776497816.204764974] [pi5_client]: Path received: 3 pts
[INFO] [1776497819.255929073] [pi5_client]: Path received: 3 pts
[INFO] [1776497822.244519273] [pi5_client]: Path received: 3 pts
[INFO] [1776497825.260182796] [pi5_client]: Path received: 3 pts
[INFO] [1776497828.298966175] [pi5_client]: Path received: 4 pts
```
