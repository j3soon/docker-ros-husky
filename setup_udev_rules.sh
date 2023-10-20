#!/bin/bash -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# The following 4 udev rules are installed when running the Husky install script.
# Ref: https://github.com/clearpathrobotics/ros_computer_setup/blob/90de83bffbe70fb65fdd649f6fe7c17cbbf88199/install.sh#L423-L432

sudo cp "$DIR/thirdparty/files/udev/41-clearpath.rules" /etc/udev/rules.d
sudo cp "$DIR/thirdparty/files/udev/41-logitech.rules" /etc/udev/rules.d
sudo cp "$DIR/thirdparty/files/udev/52-ftdi.rules" /etc/udev/rules.d
sudo cp "$DIR/thirdparty/files/udev/60-startech.rules" /etc/udev/rules.d

# The following udev rule is installed when running `apt install ros-melodic-husky-bringup`,
# which is a dependency of the `ros-melodic-husky-robot` package installed in the script below.
# Ref: https://github.com/clearpathrobotics/ros_computer_setup/blob/90de83bffbe70fb65fdd649f6fe7c17cbbf88199/install.sh#L360
# Run the following commands to see the rule destination:
#     apt-get download ros-melodic-husky-bringup
#     dpkg-deb --info ros-melodic-husky-bringup_0.4.13-1bionic.20230706.000013_amd64.deb
#     dpkg -x ros-melodic-husky-bringup_0.4.13-1bionic.20230706.000013_amd64.deb ros-melodic-husky-bringup
#     ls ./ros-melodic-husky-bringup/lib/udev/rules.d/60-ros-melodic-husky-bringup.rules
# which corresponds to the following final destination:
#     /lib/udev/rules.d/60-ros-melodic-husky-bringup.rules
# The rule file can be found in the husky/husky GitHub repo.
# Ref: https://github.com/husky/husky/blob/cf7a47e160a1c524391ea27b868e2dcc3f46de73/husky_bringup/debian/udev
# This rule will create a symlink at `/dev/prolific` when husky is connected, which is used in the `husky_base` launch file.
# Ref: https://github.com/husky/husky/blob/cf7a47e160a1c524391ea27b868e2dcc3f46de73/husky_base/launch/base.launch#L29

sudo cp "$DIR/thirdparty/files/udev/60-ros-melodic-husky-bringup.rules" /lib/udev/rules.d
sudo service udev reload
sudo service udev restart
read -p "Please (re-)connect Husky now, and Press enter to continue."
ls /dev/prolific
echo "done"
