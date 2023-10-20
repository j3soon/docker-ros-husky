#!/bin/bash -e

# The husky installation script in the Dockerfile will run `rosrun husky_bringup install`.
# Ref: https://github.com/clearpathrobotics/ros_computer_setup/blob/90de83bffbe70fb65fdd649f6fe7c17cbbf88199/install.sh#L471
# Then, husky_bringup will register a robot_upstart job.
# Refs:
# - https://github.com/husky/husky/blob/cf7a47e160a1c524391ea27b868e2dcc3f46de73/husky_bringup/scripts/install#L6
# - http://wiki.ros.org/robot_upstart
# - https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/
# Upon registration, robot_upstart will generate the helper script: `/usr/sbin/ros-start`.
# Ref: https://github.com/clearpathrobotics/robot_upstart/blob/9f341077933cdbc486e39931dbb3355e37f68f1d/src/robot_upstart/providers.py#L108-L109
# When installing direcly on host, robot_upstart will start all registered services automatically upon reboot.
# However, it doesn't seem to start the services in a docker container.
# Therefore, we need to run the start helper script manually:

docker exec -it ros-melodic-husky /ros_entrypoint.sh /usr/sbin/ros-start

# The helper script include the following command:
#     rosrun robot_upstart mklaunch $JOB_FOLDER > $LAUNCH_FILENAME
#     ...
#     setpriv --reuid root --regid root --init-groups roslaunch $LAUNCH_FILENAME &
# which will generate `/tmp/ros.launch` and run it. The launch file includes the `base.launch` file by:
#     <include file="/etc/ros/melodic/ros.d/base.launch" />
# which corresponds to the husky_base launch file.
# Refs:
# - https://github.com/husky/husky/blob/cf7a47e160a1c524391ea27b868e2dcc3f46de73/husky_base/launch/base.launch
# - http://www.clearpathrobotics.com/assets/guides/melodic/ros/Launch%20Files.html
# The launch file will launch some common services used by husky.
# Refs:
# - https://github.com/husky/husky/blob/melodic-devel/husky_control/launch/control.launch
# - https://github.com/husky/husky/blob/melodic-devel/husky_control/launch/teleop.launch
# Side note: The minimal services required to teleoperate husky can be checked by `rosnode list` and is listed as follows: 
# - /base_controller_spawner
# - /husky_node
# - /twist_mux
