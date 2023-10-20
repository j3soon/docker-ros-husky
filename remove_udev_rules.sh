#!/bin/bash -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

sudo rm /etc/udev/rules.d/41-clearpath.rules
sudo rm /etc/udev/rules.d/41-logitech.rules
sudo rm /etc/udev/rules.d/52-ftdi.rules
sudo rm /etc/udev/rules.d/60-startech.rules
sudo rm /lib/udev/rules.d/60-ros-melodic-husky-bringup.rules
sudo service udev reload
sudo service udev restart
echo "done"
