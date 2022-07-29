# qb-softhand2-tudelft

This repository is intended as a starting point for demonstrating and using the qb SoftHand (https://qbrobotics.com/product/qb-softhand-research/), in conjunction with Franka Emika Panda arms. \
The following instructions are adapted from https://bitbucket.org/qbrobotics/qbhand-ros/src/production-noetic/. More details are available there, though  there are some issues with the instructions which are highlighted in this readme. 
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

## SoftHand Examples
Once the controller is running, check the functionality by running the example scripts.
### demo.py
This will just open and close the hand in a loop. \
The hand is controlled by publishing on the `/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command` topic using `JointTrajectory` messages consisting of a single `JointTrajectoryPoint`.
### keyboard_control.py
This will allow you to control the synergy and manipulation degrees of freedom with the keyboard arrow keys. Up/down and left/right to increase/decrease synergy and manipulation repsectively. Synergy controls closure of the fingers, manipulation shifts the grasp between the thumb and other side of the hand.

## Using with the Panda Arm
First, familiarise yourself with the TUDelft Human Friendly Controllers for the Panda, see https://github.com/franzesegiovanni/franka_human_friendly_controllers. 
1. Mount the SoftHand: Remove the Panda Hand and mount the SoftHand using the adapter coupling in the box. Configure the end effector properties in the Panda Desk application; set the Mass to 0.886kg and the COM Flange Offset to x = 0, y = 0, z = 0.083m. (TODO - end effector offset)
2. Run the Panda FCI Controller: the example scripts are written to work with the Cartesian Endpoint Impedance Controller only, ie the arm is controlled by publishing an end effector goal pose to `/equilibrium_pose`. Launch a compatible controller on the Panda desktop PC, eg
```
roslaunch franka_human_friendly_controllers cartesian_variable_impedance_controller.launch robot_ip:=ROBOT_IP
```
3. Run python example scripts - see more details below. To run the scripts from your own laptop, see the section 'How to connect your PC to the network and read and send commands to the controller' at https://github.com/franzesegiovanni/franka_ros_TUD/tree/tu-delft-control. 

### arm_hand_combined_control.py
This controls the Panda endpoint and SoftHand setpoint from various input sources, which can be toggled on and off using the keyboard number keys. Most inputs can be active at the same time. The 3D mouse and control pad inputs require `spacenav_node` and `joy_node` respectively to be running (http://wiki.ros.org/spacenav_node, http://wiki.ros.org/joy)
```
Use number keys to toggle control inputs on/off:
	1. Keyboard	2. Mouse	3. 3D mouse	4. Control pad

Keyboard controls:
	Panda:
	W/S:		 x-position +/-
	A/D:		 y-position +/-
	Q/Z:		 z-position +/-
	H/F:		 x-orientation +/-
	T/G:		 y-orientation +/-
	R/Y:		 z-orientation +/-
	N/J:		 Nullspace stiffness on/off

	SoftHand:
	up/down:	 synergy +/-
	left/right:	 manipulation +/-
	PgUp/PgDown:	 toggles synergy fully open/closed
	,/.:		 toggles manipulation fully left/right

Mouse controls:
	Movement:	 X/Y position
	Scroll wheel: 	 Z position

3D mouse controls:
	Joystick:	 Panda goal pose adjustment
	L button:	 Toggle SoftHand synergy fully open/closed
	R button:	 Toggle SoftHand manipulation fully left/right

Control pad controls (use analog trigger mode):
	Left trigger:	 Control SoftHand manipulation
	Right trigger:	 Control SoftHand synergy
	Left bumper:	 Lock SoftHand manipulation
	Right bumper:	 Lock SoftHand synergy

Esc:	Quit
`:	Toggle suspending all keyboard input
```
The script uses the classes implemented in `panda_equilibrium_controller.py` and `softhand_setpt_controller.py` which can be reused to write other high level control scripts.
### traj_recorder.py
Records a trajectory (end effector pose, joint configuration, and SoftHand setpoint (if connected)) to an .npz file. Recording can be paused and resumed, with interpolation between poses inserted in the final trajectory. The trajectory will be saved in the working directory, if a string argument is supplied to the script it will be used as the filename, otherwise it will default to Date_Time. \
Eg (will save as test.npz):
```
python3 traj_recorder.py test
```
### traj_player.py
Plays back a trajectory (end effector pose and SoftHand setpoint (TBC)) saved in an .npz file, ie publishes the recorded data to the relevant command topics. It will first interpolate to the initial recorded pose from the current pose. Note that this just loops through all recorded points at a set rate, there is no time reference built into the saved trajectory. \
Eg (assuming test.npz is in the working directory):
```
python3 traj_player.py test.npz
```
