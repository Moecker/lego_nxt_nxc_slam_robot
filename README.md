# Overview

* Gmapping Library: http://wiki.ros.org/gmapping
* Python BT libary: https://github.com/Eelviny/nxt-python
* Pyhon BT Documentation: https://github.com/Eelviny/nxt-python/wiki/Tutorial
* FAQs: https://github.com/Eelviny/nxt-python/wiki/FAQ

# Installation
## This installs all required dependencies
Prerequisites: ros kinetic and gmapping

* sudo apt-get install python-bluez
---
* git clone https://github.com/pyusb/pyusb.git
* (Or use fork https://github.com/Moecker/pyusb.git)
* cd pyusb
* sudo python setup.py install
* sudo python setup.py install --user
---
* git clone https://github.com/Eelviny/nxt-python.git
* (Or use fork https://github.com/Moecker/nxt-python.git)
* git checkout python2
* cd nxt-python
* sudo python setup.py install
* sudo python setup.py install --user

# Install and Build
* git clone https://github.com/Moecker/lego_nxt_nxc_slam_robot.git
* catkin build

# Run the code
* source devel/setup.zsh
* roslaunch nxt_slam nxt_slam.launch

# Errors
Try to run python lego_nxt_nxc_slam_robot/src/nxt_slam/scripts/nxt_communication.py
