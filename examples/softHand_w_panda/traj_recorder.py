#%%
#!/usr/bin/env python
import sys
import datetime
import pathlib
import numpy as np
import quaternion # pip install numpy-quaternion
import rospy
import rosnode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from pynput import keyboard
from pynput.keyboard import KeyCode, Key

from traj_utils import interpolate_poses, interpolate_joints, interpolate_softhand

global key_in
global pose_current
global goal_current
global joint_state_current
global hand_command_current

def _on_press(key):
    global key_in 
    key_in = key

def pose_callback(pose):
    global pose_current 
    pose_current = pose

def goal_callback(goal):
    global goal_current 
    goal_current = goal

def joint_callback(joint_state):
    global joint_state_current 
    joint_state_current = joint_state

def hand_callback(hand_command):
    global hand_command_current
    hand_command_current = hand_command

#%%
if __name__ == '__main__':

    rospy.init_node('traj_recorder', anonymous=True)

    key_listener = keyboard.Listener(on_press=_on_press, suppress=False)
    key_listener.start()
    key_in = None

    arm_connected = True if ("/panda_controller_spawner" in rosnode.get_node_names()) or ("/franka_control" in rosnode.get_node_names()) else False
    hand_connected = True if "/qb_device_communication_handler" in rosnode.get_node_names() else False

    if arm_connected:
        pose_sub = rospy.Subscriber("/cartesian_pose", PoseStamped, pose_callback)
        goal_sub = rospy.Subscriber('/equilibrium_pose', PoseStamped, goal_callback)
        joint_sub = rospy.Subscriber("/joint_states", JointState, joint_callback)
        print("Panda arm detected")
    else:
        print("Panda arm not detected")
    if hand_connected:
        hand_sub = rospy.Subscriber('/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command', JointTrajectory, hand_callback)
        print("SoftHand detected")
    else:
        print("SoftHand not detected")
    if (not arm_connected) and (not hand_connected):
        print("Nothing to record, exiting")
        sys.exit()

    record_rate = 10 # Hz
    ros_record_rate = rospy.Rate(record_rate)

    if len(sys.argv) < 2:
        record_name = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        print("You didn't specify, so date_time has been used as the file name: ",record_name)
        print("Pass a string argument to the script to specify filename (next time).")
    else:
        record_name = str(sys.argv[1])
        print("Recording will be saved with filename: ", record_name)

    print("\nPress Space to start recording cartesian goal pose and joint positions.")
    while key_in != Key.space:
        pass
    key_in = None

    print("\nRecording started. Press P to pause.")
    pose_trajectory = pose_current if arm_connected else None # Note first point of pose trajectory is actual starting pose, the rest will be goal equilibrium poses
    joint_trajectory = joint_state_current if arm_connected else None
    hand_trajectory = hand_command_current if hand_connected else None
    ros_record_rate.sleep()

    while True:
        if key_in == KeyCode.from_char('p'):
            print("\nRecording paused. Press P again to resume recording. Press Space to end.")
            print("Note: saved trajectory will interpolate between pause and resume poses)")
            pose_at_pause = pose_current if arm_connected else None
            joints_at_pause = joint_state_current if arm_connected else None
            setpts_at_pause = hand_command_current if hand_connected else None

            key_in = None
            while (key_in != KeyCode.from_char('p') and key_in != Key.space):
                pass
            if key_in == Key.space:
                break
            elif key_in == KeyCode.from_char('p'):
                # Insert interpolations into trajectories
                interp_length = 0
                if arm_connected:
                    interpolated_poses = interpolate_poses(pose_at_pause, pose_current, 0.1/record_rate) # 0.1m/s
                    pose_trajectory = np.c_[pose_trajectory, interpolated_poses]
                    interp_length = interpolated_poses.size
                    interpolated_joints = interpolate_joints(joints_at_pause, joint_state_current, np.pi/4/record_rate) # 45deg/s
                    joint_trajectory = np.c_[joint_trajectory, interpolated_joints]
                    interp_length = np.max([interpolated_joints.size, interp_length])
                if hand_connected:
                    interpolated_setpts = interpolate_softhand(setpts_at_pause, hand_command_current, 0.5/record_rate) # close halfway/s 
                    hand_trajectory = np.c_[hand_trajectory, interpolated_setpts]
                    interp_length = np.max([interpolated_setpts.size, interp_length])
                # Add padding so interpolation lengths match
                if arm_connected:
                    for n_pad in range(interp_length-interpolated_poses.size):
                        pose_trajectory = np.c_[pose_trajectory, pose_trajectory[0][-1]]
                    for n_pad in range(interp_length-interpolated_joints.size):
                        joint_trajectory = np.c_[joint_trajectory, joint_trajectory[0][-1]]
                if hand_connected:
                    for n_pad in range(interp_length-interpolated_setpts.size):
                        hand_trajectory = np.c_[hand_trajectory, hand_trajectory[0][-1]]

                key_in = None
                print("\nRecording resumed. Press P to pause.")
        else:
            # Recording
            if arm_connected:
                pose_trajectory = np.c_[pose_trajectory, goal_current]  # TODO - should record current pose instead of goal?
                joint_trajectory = np.c_[joint_trajectory, joint_state_current]
            if hand_connected: hand_trajectory = np.c_[hand_trajectory, hand_command_current]
            ros_record_rate .sleep()
    
    np.savez(
        str(pathlib.Path().resolve())+'/'+str(record_name)+'.npz',
        pose_trajectory = pose_trajectory,
        joint_trajectory = joint_trajectory,
        hand_trajectory = hand_trajectory
    )
    print("Trajectory data saved to: ", str(pathlib.Path().resolve())+'/'+str(record_name)+'.npz')
