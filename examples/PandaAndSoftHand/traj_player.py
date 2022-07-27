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

# Adapted from Learning_from_demonstration.py go_to_pose() TODO - maybe refactor a bit
def go_to_pose(goal_pose):
    start = np.array([pose_current.pose.position.x, pose_current.pose.position.y, pose_current.pose.position.z])
    start_ori = np.array([pose_current.pose.orientation.w, pose_current.pose.orientation.x, pose_current.pose.orientation.y, pose_current.pose.orientation.z])
    goal_ = np.array([goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z])
    # interpolate from start to goal with attractor distance of approx 10 cm (changed from 1cm to account for slower record rate 10Hz)
    squared_dist = np.sum(np.subtract(start, goal_)**2, axis=0)
    dist = np.sqrt(squared_dist)
    interp_dist = 0.01  # [m]
    step_num_lin = math.floor(dist / interp_dist)
    
    q_start=np.quaternion(start_ori[0], start_ori[1], start_ori[2], start_ori[3])
    q_goal=np.quaternion(goal_pose.pose.orientation.w, goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z)
    inner_prod=q_start.x*q_goal.x+q_start.y*q_goal.y+q_start.z*q_goal.z+q_start.w*q_goal.w
    if inner_prod < 0:
        q_start.x=-q_start.x
        q_start.y=-q_start.y
        q_start.z=-q_start.z
        q_start.w=-q_start.w
    inner_prod=q_start.x*q_goal.x+q_start.y*q_goal.y+q_start.z*q_goal.z+q_start.w*q_goal.w
    theta= np.arccos(np.abs(inner_prod))
    interp_dist_polar = 0.01 
    step_num_polar = math.floor(theta / interp_dist_polar)

    step_num=np.max([step_num_polar,step_num_lin])

    x = np.linspace(start[0], goal_pose.pose.position.x, step_num)
    y = np.linspace(start[1], goal_pose.pose.position.y, step_num)
    z = np.linspace(start[2], goal_pose.pose.position.z, step_num)
    
    goal = PoseStamped()
    
    goal.pose.position.x = x[0]
    goal.pose.position.y = y[0]
    goal.pose.position.z = z[0]
    
    quat=np.slerp_vectorized(q_start, q_goal, 0.0)
    goal.pose.orientation.x = quat.x
    goal.pose.orientation.y = quat.y
    goal.pose.orientation.z = quat.z
    goal.pose.orientation.w = quat.w

    goal_pub.publish(goal)

    goal = PoseStamped()
    for i in range(step_num):      
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = x[i]
        goal.pose.position.y = y[i]
        goal.pose.position.z = z[i]
        quat=np.slerp_vectorized(q_start, q_goal, i/step_num)
        goal.pose.orientation.x = quat.x
        goal.pose.orientation.y = quat.y
        goal.pose.orientation.z = quat.z
        goal.pose.orientation.w = quat.w
        goal_pub.publish(goal)
        record_rate.sleep()


#%%
if __name__ == '__main__':

    rospy.init_node('traj_player', anonymous=True)

    pose_sub = rospy.Subscriber("/cartesian_pose", PoseStamped, pose_callback)
    goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
    goal_pose = PoseStamped()
    
    key_listener = keyboard.Listener(on_press=_on_press, suppress=False)
    key_listener.start()
    key_in = None

    record_rate = rospy.Rate(10)

    if len(sys.argv) < 2:
        print("Path to recorded trajectory (relative to script) must be included as an argument.")
        exit
    record = np.load(str(pathlib.Path().resolve())+'/'+sys.argv[1], allow_pickle=True)

    playback_data = record['pose_trajectory'].squeeze()
    print("Going to starting pose...")
    go_to_pose(playback_data[0])
    print("Playback started. Press Esc to stop.")
    for i in range (playback_data.size):
        goal_pose = playback_data[i]
        goal_pub.publish(goal_pose)
        if key_in == Key.esc:
            break
        record_rate.sleep()
    print("Playback completed.")
    