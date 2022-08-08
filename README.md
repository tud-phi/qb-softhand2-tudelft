# qb-softhand2-tudelft

This repository is intended as a starting point for demonstrating and using the qb SoftHand (https://qbrobotics.com/product/qb-softhand-research/), in conjunction with Franka Emika Panda arms. \
The following instructions are adapted from https://bitbucket.org/qbrobotics/qbhand-ros/src/production-noetic/. More details are available there, though  there are some issues with the instructions which are highlighted in this readme. 
## Install SoftHand ROS Packages
If installation from Ubuntu repositories (ie `sudo apt install ros-<ros_distro>-qb-hand`) as mentioned in the bitbucket instructions doesn't appear to work, the packages can be installed from source instead:
1. Clone the qb_device and qb_hand repos in your catkin workspace (note the additional option needed for qb_device, see https://bitbucket.org/qbrobotics/qbdevice-ros/issues/4/cmakelists-missing)
```
git clone https://bitbucket.org/qbrobotics/qbdevice-ros --recurse-submodules
git clone https://bitbucket.org/qbrobotics/qbhand-ros.git
```
2. Compile with `catkin_make` or `catkin build`. You may need some extra packages to properly compile the code. Please, be sure that you have already installed at least ros-<ros_distro>-ros-controllers, ros-<ros_distro>-transmission-interface, ros-<ros_distro>-joint-limits-interface, ros-<ros_distro>-combined-robot-hw, and their dependencies. Check the build output for error info.

## Start the SoftHand controller
1. If you have never set it up, you probably need to add your linux user to the `dialout` group to grant right access to the serial port resources. Execute the following in a terminal: `sudo gpasswd -a <linux_user_name> dialout`; you may need to reboot afterwards. 
2. Connect the SoftHand's cable to the power supply and a USB port on your PC. See the documentation for details.
3. To bring the SoftHand controller online, use the below terminal command. Note that the launch file name is different from the bitbucket instructions.
```
roslaunch qb_hand_control control_qbhand2m.launch standalone:=true activate_on_initialization:=true device_id:=<actual_device_id>
```
Generally, `<actual_device_id>` should be 1. If this isn't working, you may need to run the GUI application from the qb website to see if it has been changed.

## SoftHand Examples
Once the controller is running, check the functionality by running the example scripts.
### demo.py
This will just open and close the hand in a loop. \
The hand is controlled by publishing on the `/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command` topic using `JointTrajectory` messages consisting of a single `JointTrajectoryPoint`. [^1]
### keyboard_control.py
This will allow you to control the synergy and manipulation degrees of freedom with the keyboard arrow keys. Up/down and left/right to increase/decrease synergy and manipulation repsectively. Synergy controls closure of the fingers, manipulation shifts the grasp between the thumb and other side of the hand.

## Using with the Panda Arm
First, familiarise yourself with the TUDelft Human Friendly Controllers for the Panda, see https://github.com/franzesegiovanni/franka_human_friendly_controllers. 
1. Mount the SoftHand: Remove the Panda Hand and mount the SoftHand using the adapter coupling in the box.[^2] In the Panda Desk application go to Settings>End Effector and upload the `qbSoftHand_eeConfig.json` configuration file (or manually set the Mass to 1.0kg and the COM Flange Offset to x = 0, y = 0, z = 0.083m).
2. Run the Panda FCI Controller: the example scripts are written to work with the Cartesian Endpoint Impedance Controller only, ie the arm is controlled by publishing an end effector goal pose to `/equilibrium_pose`. Launch a compatible controller on the Panda desktop PC, eg
```
roslaunch franka_human_friendly_controllers cartesian_variable_impedance_controller.launch robot_ip:=<ROBOT_IP>
```
3. Run python example scripts - see more details below. To run the scripts from your own laptop, see the section 'How to connect your PC to the network and read and send commands to the controller' at https://github.com/franzesegiovanni/franka_ros_TUD/tree/tu-delft-control. 

### arm_hand_combined_control.py
This controls the Panda endpoint and SoftHand setpoint from various input sources, which can be toggled on and off using the keyboard number keys. Most inputs can be active at the same time. All keyboard input can be toggled off with \` (if you remember to) so that you can type elsewhere. The 3D mouse and control pad inputs[^3] require `spacenav_node` and `joy_node` respectively to be running (http://wiki.ros.org/spacenav_node, http://wiki.ros.org/joy). \
The script turns impedance stiffness off when it is launched, so the arm can be moved manually in gravity compensation mode. The script constantly resets the impedance controller goal to the current pose in this mode to avoid them separating and causing sudden movements if the stiffness is turned back on, however care should still be taken.
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
	9/0:		 Impedance stiffness on/off
	N/J:		 Nullspace stiffness on/off

	SoftHand:
	up/down:	 synergy +/-
	left/right:	 manipulation +/-
	PgUp/PgDown:	 toggle synergy fully open/closed
	,/.:		 toggle manipulation fully left/right

Mouse controls:
	Movement:	 X/Y position
	Scroll wheel: 	 Z position

3D mouse controls:
	Joystick:	 Panda goal pose adjustment
	L button:	 Toggle SoftHand synergy fully open/closed
	R button:	 Toggle SoftHand manipulation fully left/right

Control pad controls (use analog trigger mode):
	Left thumbstick:	 move X & Y direction in end effector frame
	Right thumbstick:	 rotate around X & Y axes in end effector frame
	D-pad Left/Right:	 rotate around Z zaxis in end effector frame
	D-pad Up/Down:		 move Z direction in end effector frame
	Left trigger:		 Control SoftHand manipulation
	Right trigger:		 Control SoftHand synergy
	Left bumper:		 Lock/Unlock SoftHand manipulation
	Right bumper:		 Lock/Unlock SoftHand synergy

Esc:	Quit
`:	Toggle suspending all keyboard input
```
The script uses the classes implemented in `panda_equilibrium_controller.py` and `softhand_setpt_controller.py`.
### traj_recorder.py
Records a trajectory (end effector pose, joint configuration, and SoftHand setpoint) to an .npz file. Recording can be paused and resumed, with interpolation between discontinuities automatically inserted in the final trajectory. The trajectory will be saved in the working directory; if a string argument is supplied to the script it will be used as the filename, otherwise it will default to Date_Time. \
Eg (will save as test.npz):
```
python3 traj_recorder.py test
```
### traj_player.py
Plays back a trajectory (end effector pose and SoftHand setpoint) saved in an .npz file, ie publishes the recorded data to the relevant command topics. It will first interpolate to the initial recorded pose from the current pose. Note that this just loops through all recorded points at a set rate, there is no time reference built into the saved trajectory. \
Eg (assuming test.npz is in the working directory):
```
python3 traj_player.py test.npz
```
There are some example recorded trajectories in the examples folder.

[^1]: This follows how the setpoint is controlled when using qb's GUI control node. Note that since each setpoint is sent seperately, a SoftHand trajectory is sent as a sequence of single point `JointTrajectory` messages. Additionally, each `JointTrajectoryPoint` requires a non-zero 'time_from_start' (otherwise the setpoint is in the past once it reaches the SoftHand controller). There may be a better way to command the SoftHand controller that avoids these issues.
[^2]: The hand can be mounted in 8 wrist orientations. For the example trajectories it was mounted so that the black coloured countersink in the hand adapter lines up with the white filled screw head on the end effector flange (for Panda IP 172.16.0.2).
[^3]: There are some idiosyncrasies with the control pad control. Firstly, constant joystick inputs are not automatically repeated by ROS. This can be enabled by passing the parameter `_autorepeat_rate=<Hz>` when running `joy_node` (note the rate will affect how fast the endpoint moves). Secondly, the control scheme has been implemented in the end effector frame - the left stick moves side to side and up and down (up being the direction the thumb points when the hand is open) and the right stick rotates around these axes. The D-pad corresponds to the Z-axis - forward/backward movement and rotation around the wrist. You may or may not find this scheme intuitive.
