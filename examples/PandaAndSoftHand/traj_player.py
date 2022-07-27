#%%
#!/usr/bin/env python
import sys
import pathlib
import math
import numpy as np
import quaternion # pip install numpy-quaternion
import rospy
from geometry_msgs.msg import PoseStamped
from pynput import keyboard
from pynput.keyboard import Key

global key_in
global pose_current

def _on_press(key):
    global key_in 
    key_in = key

def pose_callback(pose):
    global pose_current 
    pose_current = pose

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

    rospy.init_node('traj_player', anonymous=True)

    pose_sub = rospy.Subscriber("/cartesian_pose", PoseStamped, pose_callback)
    goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
    goal_pose = PoseStamped()
    
    key_listener = keyboard.Listener(on_press=_on_press, suppress=False)
    key_listener.start()
    key_in = None

    record_rate = 10 # Hz
    ros_record_rate = rospy.Rate(record_rate)
    
    if len(sys.argv) < 2:
        print("Path to recorded trajectory (relative to script) must be included as an argument.")
        sys.exit()
    record = np.load(str(pathlib.Path().resolve())+'/'+sys.argv[1], allow_pickle=True)
    playback_data = record['pose_trajectory'].squeeze()

    print("Going to starting pose...")
    traj_to_start = interpolate_poses(pose_current, playback_data[0], 0.1/record_rate).squeeze()
    for i in range (traj_to_start.size):
        goal_pose = traj_to_start[i]
        goal_pub.publish(goal_pose)
        if key_in == Key.esc:
            break
        ros_record_rate.sleep()

    print("Playback started. Press Esc to stop.")
    for i in range (playback_data.size):
        goal_pose = playback_data[i]
        goal_pub.publish(goal_pose)
        if key_in == Key.esc:
            break
        ros_record_rate.sleep()

    print("Playback completed.")
    