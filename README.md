# Robotics Project: CheckPointNavigationRosmasterx3

## Summary

- [Introduction](#Introduction)
- [Installazione](#Installation-guide)
- [Start mapping](#start-mapping)
- [Start patrolling](#start-patrolling)

---

## Introduction

This project aims to map a location with using lidar and slam_toolbox and then navigating the mapped envinronment.
The navigation works by setting "checkpoints" in the map, and then the robot will find the optimal path and reach it.

## Installation guide

- This project is designed to run on a docker installed on a yahboom rosmasterx3
- Connect to the rosmasterx3 using ssh, the ip address is shown on the display
- [Download Docker](https://www.docker.com)
- To setup run the command

```bash
  docker build -t ros2-humble-arm64 .
```

in the folder where the Dockerfile is located.

## Start mapping

- Connect to the rosmasterx3 using ssh
- Open the docker with the command

```bash
   docker run -it --device=/dev/myserial --device=/dev/rplidar ros2-humble-arm64
```

-Run the command

```bash
   ros2 launch mapping_ws start_mapping.py 
```

- Move the robot in the environment by using the keyboard.
- When you're done press ctrl+c, this will close the program and save the map.

## Start patrolling

- Connect to the rosmasterx3 using ssh
- Open the docker with the command

```bash
   docker run -it --device=/dev/myserial --device=/dev/rplidar ros2-humble-arm64
```

- Run the command

```bash
   ros2 launch patrol_ws start_patrolling.py 
```
