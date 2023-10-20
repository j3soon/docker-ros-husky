#!/bin/bash -e

docker exec -it ros-melodic-husky /ros_entrypoint.sh \
    bash -c "source ./devel/setup.bash && rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
