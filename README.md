# qb-softhand2-tudelft

## Install SoftHand ROS Packages
https://bitbucket.org/qbrobotics/qbhand-ros/src/production-melodic/
- install sudo apt ros-noetic-qb-hand not working
- need to install packages from source (from above link)
1. Clone qb_device and qb_hand packages in your catkin workspace (note additional option needed for qbdevice, see https://bitbucket.org/qbrobotics/qbdevice-ros/issues/4/cmakelists-missing)
```
git clone https://bitbucket.org/qbrobotics/qbdevice-ros --recurse-submodules
git clone https://bitbucket.org/qbrobotics/qbhand-ros.git
```
2. Compile with `catkin_make` or `catkin build`

Note: depending on your ROS installation, you may need some extra packages to properly compile the code. Please, be sure that you have already installed at least ros-kinetic-ros-controllers, ros-kinetic-transmission-interface, ros-kinetic-joint-limits-interface, ros-kinetic-combined-robot-hw, and their dependencies (e.g. use sudo apt install <ros-pkg>).

## Run the SoftHand Demos

```
roslaunch qb_hand_control control_qbhand2m.launch standalone:=true activate_on_initialization:=true device_id:=<actual_device_id>
```
- <actual_device_id> should be 1 (if not working check in GUI)
- note launch file name is different than in bitbucket instructions
