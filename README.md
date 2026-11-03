# slam-rc-car

an rc car that maps a room with lidar + imu, finds apriltags, and then drives a treasure-hunt route by itself (start → checkpoint 1 → checkpoint 2 → treasure → finish), playing a sound at each tag. ros 2 + cartographer running on a raspberry pi 5.

final project for ece 302 at princeton, with aaryan jagtap.

<p align="center"><img src="docs/car.jpg" alt="the car: rplidar on top, pi 5, pi camera on the front, speaker in the back" width="420"></p>

## how it fits together

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/nodes-dark.svg">
  <img alt="ros2 node graph: lidar, imu and camera feed cartographer and the apriltag node; cartographer's map feeds an a* planner that drives the motor controller" src="docs/nodes-light.svg">
</picture>

- **mapping:** cartographer fuses `/scan` from the rplidar a1 with `/imu` from a bno055. the saved map covers about 11 × 15 m at 5 cm per cell ([`maps/`](maps)).
- **landmarks:** the pi camera spots 36h11 apriltags and gets each pose with pnp from the four corners, averaged over several sightings. tags go to cartographer as `/landmarks` for loop closure and get stored as waypoints in the `map` frame.
- **driving:** a custom a* planner runs on the occupancy grid with 0.2 m of obstacle inflation, smooths the path and publishes `/cmd_vel` from a 10 hz control loop. the motor node turns that into 50 hz pwm for the esc and steering servo through `lgpio`, and stops the car if commands go quiet for 0.5 s.

| apriltag detection | a* to the first checkpoint |
| --- | --- |
| <img src="docs/apriltag.jpg" alt="tag id 0 labeled start, 0.22 m away" width="400"> | <img src="docs/astar.png" alt="planned path from start to checkpoint 1 on the occupancy grid" width="300"> |

## decisions worth noting

- **no lidar-only odometry.** rf2o (laser scan matching) drifted in featureless hallways and at the a1's slow scan rate. cartographer with the imu as the tracking frame fixed it: the imu covers the gaps between scans and the lidar corrects the drift.
- **tags for loop closure.** scan matching gets confused by repetitive rooms. a tag with a unique id is an unambiguous "i've been here", so revisits trigger pose-graph optimization reliably.
- **ubuntu instead of raspberry pi os**, to get the full ros 2 package set (cartographer, tf2, foxglove bridge).
- **the esc is inverted:** 100% duty is stop and 85% is full speed. the servo centers at 7.5% duty with ±2.5% of travel.

## hardware

| part | what |
| --- | --- |
| compute | raspberry pi 5, 8 gb |
| lidar | slamtec rplidar a1 (360°, ~6 m) |
| imu | bno055, 9-dof |
| camera | pi camera v3 |
| drive | brushless esc + steering servo |

## run it

on a pi 5 with ubuntu 24.04 and ros 2 humble:

```bash
sudo apt install ros-humble-desktop ros-humble-cartographer-ros ros-humble-foxglove-bridge
pip3 install lgpio apriltag opencv-python numpy

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/aarushgoradia/slam-rc-car.git
cd ~/ros2_ws && colcon build && source install/setup.bash
```

```bash
# 1. map: start cartographer, then tag logging + the map saver, then drive it around by hand
ros2 launch rc_car_slam cartographer.launch.py
ros2 launch rc_car_slam mapping_mode.launch.py

# 2. run the route: localize on the saved map and drive the waypoints
ros2 launch rc_car_slam autonomous_mode.launch.py
```

the launch files and nodes point at `/home/bench207/ros2_ws/...` for the map, the tag file and the sounds. change those paths to match your machine.

## what's where

| path | what |
| --- | --- |
| `src/rc_car_sensors/` | imu, camera + apriltag, motor controller, a* planner, autonomous nav nodes |
| `src/rc_car_slam/` | cartographer configs and the launch files |
| `src/rc_car_description/` | urdf + static transforms (`map → odom → base_link → laser / imu_link / camera_link`) |
| `src/sllidar_ros2/` | slamtec's lidar driver (submodule) |
| `maps/` | the saved map (`.pgm` + `.yaml` + cartographer `.pbstream`) |
| `sounds/` | the five checkpoint sounds |

mit license.
