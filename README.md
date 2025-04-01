# Docker image for Clearpath's Husky A200 on ROS 1 Melodic

[<img src="https://img.shields.io/badge/dockerhub-image-important.svg?logo=docker">](https://hub.docker.com/r/j3soon/ros-melodic-husky/tags)

> Although this repo should still function as intended, future updates for ROS 2 will be maintained at [j3soon/ros2-essentials](https://github.com/j3soon/ros2-essentials/tree/main/husky_ws).

## Prerequisites

Hardware:

- Husky base
- Power supply cable (for recharging the battery)
- USB cable

We choose not to use the MINI-ITX computer, and control Husky directly through a Jetson board or laptop.

More information such as User Guide and Manual Installation steps can be found in [this post](https://j3soon.com/cheatsheets/clearpath-husky/).

## Installation

Clone the repo:

```sh
git clone https://github.com/j3soon/docker-ros-husky.git
cd docker-ros-husky
```

Installation of udev rules must be done on the host machine:

```sh
./setup_udev_rules.sh
```

You should see `done` if everything works correctly.

You need to reboot the host machine to make the udev rules take effect.

## Teleoperation

```sh
sudo apt-get update && sudo apt-get install -y docker.io docker-compose
# Connect and power on husky
docker-compose up -d
./docker-exec-bringup.sh
# Open a new terminal
./docker-exec-teleop.sh
# Control husky with keyboard
# Press Ctrl+C to exit
docker-compose down
```

The [pre-built docker images](https://hub.docker.com/r/j3soon/ros-melodic-husky/tags) will be pulled automatically.

Although the docker container support hot plugging, if Husky is re-plugged or re-booted, you still need to re-run the following commands:

```sh
./docker-exec-bringup.sh
./docker-exec-teleop.sh
```

## Build Docker Images Locally

- On amd64 machine:

  ```sh
  docker build -f Dockerfile -t j3soon/ros-melodic-husky:latest .
  ```

- On arm64 machine:

  ```sh
  docker build -f Dockerfile.jetson -t j3soon/ros-melodic-husky:latest .
  ```

If you want to build an image that supports multiple architectures, please refer to the [build workflow](./.github/workflows/build.yaml).

## Third Party Scripts

- `thirdparty/files/udev/60-ros-melodic-husky-bringup.rules` is copied from: <https://github.com/husky/husky/blob/melodic-devel/husky_bringup/debian/udev> (commit cf7a47e)
- `thirdparty/50-clearpath.list` is copied from: <https://github.com/clearpathrobotics/public-rosdistro/blob/master/rosdep/50-clearpath.list> (commit 1e88245)
- `thirdparty/*` is copied from: <https://github.com/clearpathrobotics/ros_computer_setup/tree/main> (commit 90de83b)
  - `thirdparty/install.sh` is further modified to meet our needs.

## Uninstall

Uninstallation of udev rules must be done on the host machine:

```sh
./remove_udev_rules.sh
```

You should see `done` if everything works correctly.

## Tests

Last tested manually on 2023/10/26:

- Ubuntu 18.04.6 LTS (amd64) on Intel CPU

Last tested manually on 2023/10/27:

- Ubuntu 20.04.6 LTS (arm64) on Jetson AGX Xavier (Jetpack 5.1.2)

## Troubleshooting

- Most command failures can be resolved by simply re-running the command or rebooting Husky.
- Exec into the container with bash for debugging:
  ```sh
  ./docker-exec-debug.sh
  ```
- See [this post](https://j3soon.com/cheatsheets/robot-operating-system/) for troubleshooting ROS.
- See [this post](https://j3soon.com/cheatsheets/clearpath-husky/) for troubleshooting Husky.
