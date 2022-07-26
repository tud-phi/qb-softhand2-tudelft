# qb-softhand2-tudelft

This repository is intended as a starting point for demonstrating and using the qb SoftHand (https://qbrobotics.com/product/qb-softhand-research/), in conjunction with Franka Emika Panda arms. \
The following instructions are adapted from https://bitbucket.org/qbrobotics/qbhand-ros/src/production-melodic/. More details are available there, though  there are some issues with the instructions which are highlighted in this readme. 
## Install SoftHand ROS Packages
Installation from Ubuntu repositories (ie `sudo apt install ros-<ros_distro>-qb-hand`) as mentioned in the bitbucket instructions doesn't appear to work. Instead, the packages can be installed from source:
1. Clone the qb_device and qb_hand repos in your catkin workspace (note the additional option needed for qb_device, see https://bitbucket.org/qbrobotics/qbdevice-ros/issues/4/cmakelists-missing)
```
git clone https://bitbucket.org/qbrobotics/qbdevice-ros --recurse-submodules
git clone https://bitbucket.org/qbrobotics/qbhand-ros.git
```
2. Compile with `catkin_make` or `catkin build`. You may need some extra packages to properly compile the code. Please, be sure that you have already installed at least ros-<ros_distro>-ros-controllers, ros-<ros_distro>-transmission-interface, ros-<ros_distro>-joint-limits-interface, ros-<ros_distro>-combined-robot-hw, and their dependencies. Check the build output for error info.

## Start the SoftHand controller
Connect the SoftHand's cable to the power supply and a USB port on your PC. See the documentation for details. \
To bring the SoftHand controller online, use the below terminal command. Note that the launch file name is different from the bitbucket instructions.
```
roslaunch qb_hand_control control_qbhand2m.launch standalone:=true activate_on_initialization:=true device_id:=<actual_device_id>
```
Generally, `<actual_device_id>` should be 1. If this isn't working, you may need to run the GUI application from the qb website to see if it has been changed.

## Examples
Once the controller is running, check the functionality by running the example scripts.
### demo.py
This will just open and close the hand in a loop.
### keyboard_control.py
This will allow you to control the synergy and manipulation degrees of freedom with the keyboard arrow keys. Up/down and left/right to increase/decrease synergy and manipulation repsectively. Synergy controls closure of the fingers, manipulation shifts the grasp between the thumb and other side of the hand.
