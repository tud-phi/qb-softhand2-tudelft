#%%
#!/usr/bin/env python
import sys
import datetime
import pathlib
import numpy as np
import quaternion # pip install numpy-quaternion
import math
import rospy
import rosnode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from pynput import keyboard
from pynput.keyboard import KeyCode, Key

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

# Adapted from Learning_from_demonstration.py go_to_pose()
def interpolate_poses(pose_start, pose_end, interp_dist):
    start_pos = np.array([pose_start.pose.position.x, pose_start.pose.position.y, pose_start.pose.position.z])
    start_quat = np.quaternion(pose_start.pose.orientation.w, pose_start.pose.orientation.x, pose_start.pose.orientation.y, pose_start.pose.orientation.z)
    goal_pos = np.array([pose_end.pose.position.x, pose_end.pose.position.y, pose_end.pose.position.z])
    goal_quat = np.quaternion(pose_end.pose.orientation.w, pose_end.pose.orientation.x, pose_end.pose.orientation.y, pose_end.pose.orientation.z)

    # Determine no. of interpolation steps 
    # Distance steps
    squared_dist = np.sum(np.subtract(start_pos, goal_pos)**2, axis=0)
    dist = np.sqrt(squared_dist)
    interp_dist = interp_dist  # [m]
    step_num_lin = math.floor(dist / interp_dist)
    # Orientation steps
    inner_prod = start_quat.x*goal_quat.x+start_quat.y*goal_quat.y+start_quat.z*goal_quat.z+start_quat.w*goal_quat.w
    if inner_prod < 0:
        start_quat.x = -start_quat.x
        start_quat.y = -start_quat.y
        start_quat.z = -start_quat.z
        start_quat.w = -start_quat.w
    inner_prod = start_quat.x*goal_quat.x+start_quat.y*goal_quat.y+start_quat.z*goal_quat.z+start_quat.w*goal_quat.w
    theta = np.arccos(np.abs(inner_prod))
    interp_dist_polar = 0.01 
    step_num_polar = math.floor(theta / interp_dist_polar)
    # Use maximum step count
    step_num = np.max([step_num_polar,step_num_lin])

    # Construct list of interpolated poses
    interpolated_traj = pose_start

    interp_x = np.linspace(start_pos[0], goal_pos[0], step_num)
    interp_y = np.linspace(start_pos[1], goal_pos[1], step_num)
    interp_z = np.linspace(start_pos[2], goal_pos[2], step_num)

    for i in range(step_num):
        interp_pose = PoseStamped()      
        interp_pose.header.seq = 1
        interp_pose.header.stamp = rospy.Time.now()
        interp_pose.header.frame_id = ""

        interp_pose.pose.position.x = interp_x[i]
        interp_pose.pose.position.y = interp_y[i]
        interp_pose.pose.position.z = interp_z[i]
        interp_quat = np.slerp_vectorized(start_quat, goal_quat, i/step_num)
        interp_pose.pose.orientation.x = interp_quat.x
        interp_pose.pose.orientation.y = interp_quat.y
        interp_pose.pose.orientation.z = interp_quat.z
        interp_pose.pose.orientation.w = interp_quat.w

        interpolated_traj = np.c_[interpolated_traj, interp_pose]

    return interpolated_traj

#%%
if __name__ == '__main__':

    rospy.init_node('traj_recorder', anonymous=True)

    if "qb_device_communication_handler" in rosnode.get_node_names():
        record_hand = True
    else:
        print("SoftHand connection not detected, recording arm trajectory only.")
        record_hand = False

    pose_sub = rospy.Subscriber("/cartesian_pose", PoseStamped, pose_callback)
    goal_sub = rospy.Subscriber('/equilibrium_pose', PoseStamped, goal_callback)
    joint_sub = rospy.Subscriber("/joint_states", JointState, joint_callback)
    if record_hand: hand_sub = rospy.Subscriber('/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command', JointTrajectory, hand_callback)

    key_listener = keyboard.Listener(on_press=_on_press, suppress=False)
    key_listener.start()
    key_in = None

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
    pose_trajectory = pose_current  # Note first point of pose trajectory is actual starting pose, the rest will be goal equilibrium poses
    joint_trajectory = joint_state_current
    hand_trajectory = hand_command_current if record_hand else None
    ros_record_rate.sleep()

    while True:
        if key_in == KeyCode.from_char('p'):
            print("\nRecording paused. Press P again to resume recording. Press Space to end.")
            print("Note: saved trajectory will interpolate between pause and resume poses)")
            pose_at_pause = pose_current
            key_in = None
            while (key_in != KeyCode.from_char('p') and key_in != Key.space):
                pass
            if key_in == Key.space:
                break
            elif key_in == KeyCode.from_char('p'):
                interpolated_traj = interpolate_poses(pose_at_pause, pose_current, 0.1/record_rate)
                pose_trajectory = np.c_[pose_trajectory, interpolated_traj] # TODO - add joint and hand interpolation
                key_in = None
                print("\nRecording resumed. Press P to pause.")
        else:
            # Recording
            pose_trajectory = np.c_[pose_trajectory, goal_current]  # TODO - should record current pose instead of goal?
            joint_trajectory = np.c_[joint_trajectory, joint_state_current]
            if record_hand: hand_trajectory = np.c_[hand_trajectory, hand_command_current]
            ros_record_rate.sleep()
    
    np.savez(
        str(pathlib.Path().resolve())+'/'+str(record_name)+'.npz',
        pose_trajectory = pose_trajectory,
        joint_trajectory = joint_trajectory,
        hand_trajectory = hand_trajectory
    )
    print("Trajectory data saved to: ", str(pathlib.Path().resolve())+'/'+str(record_name)+'.npz')
