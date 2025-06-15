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
  docker run -it --rm  -p 8080:8080 --privileged -v /dev:/dev ros2-humble-arm64
```

-Run the commands

```bash
   ros2 launch mapping mapping_launch.py
   ros2 run mapping keyboard.py
```

- Move the robot in the environment by using the keyboard.
- When you're done press ctrl+c or q, this will close the program and save the map.

## Start patrolling

- Connect to the rosmasterx3 using ssh
- Open the docker with the command

```bash
  docker run -it --privileged -v /dev:/dev ros2-humble-arm64
```

- Run the command

```bash
   ros2 run client_controller client_controller
   ros2 run driver_server driver_server
```
## Open the camera
- If needed you can use
```bash
   ros2 launch astra_camera astro_pro_plus.launch.xml
   ros2 run web_video_server web_video_server
```
to activate and stream what is shown on the mounted camera.
- Open a browser and search http://YOUR-ROBOT-IP/stream?topic=/camera/color/image_raw to see the stream