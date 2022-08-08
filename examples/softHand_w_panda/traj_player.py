#%%
#!/usr/bin/env python
import sys
import pathlib
import numpy as np
#import quaternion # pip install numpy-quaternion
import rospy
import rosnode
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from pynput import keyboard
from pynput.keyboard import Key

from traj_utils import interpolate_poses

global key_in
global pose_current

def _on_press(key):
    global key_in 
    key_in = key

def pose_callback(pose):
    global pose_current 
    pose_current = pose

def hand_callback(hand_command):
    global hand_command_current
    hand_command_current = hand_command

#%%
if __name__ == '__main__':

    rospy.init_node('traj_player', anonymous=True)

    # Avoid conflicts between playback and manual control
    if ("/arm_hand_combined_control" in rosnode.get_node_names()):
        print("Please shut down manual control before starting trajectory playback.")
        sys.exit()

    # Initialisation and hardware detection
    key_listener = keyboard.Listener(on_press=_on_press, suppress=False)
    key_listener.start()
    key_in = None

    arm_connected = True if ("/panda_controller_spawner" in rosnode.get_node_names()) or ("/franka_control" in rosnode.get_node_names()) else False # panada_controller_spawner is for Gazebo
    hand_connected = True if "/qb_device_communication_handler" in rosnode.get_node_names() else False

    if arm_connected:
        pose_sub = rospy.Subscriber("/cartesian_pose", PoseStamped, pose_callback)
        pose_current = None
        goal_pub = rospy.Publisher("/equilibrium_pose", PoseStamped, queue_size=0)
        goal_pose = PoseStamped()
        print("Panda arm detected")
    else:
        print("Panda arm not detected")
    if hand_connected:
        hand_pub = rospy.Publisher('/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command', JointTrajectory, queue_size=0)
        hand_setpt = JointTrajectory()
        print("SoftHand detected")
    else:
        print("SoftHand not detected")
    if (not arm_connected) and (not hand_connected):
        print("Nothing to play to, exiting")
        sys.exit()

    record_rate = 30 # Hz. Should match recording rate.
    ros_record_rate = rospy.Rate(record_rate)
    
    # Load data and check which trajectories are contained
    if len(sys.argv) < 2:
        print("Path to recorded trajectory (relative to script) must be included as an argument.")
        sys.exit()
    record = np.load(str(pathlib.Path().resolve())+'/'+sys.argv[1], allow_pickle=True)
    pose_data = record['pose_trajectory'].squeeze()
    if np.any(pose_data) == None:
        print("Recorded Panda pose trajectory is empty")
        arm_connected = False
    else:
        record_length = pose_data.size
    hand_data = record['hand_trajectory'].squeeze()
    if np.any(hand_data) == None: 
        print("Recorded SoftHand trajectory is empty")
        hand_connected = False
    else:
        record_length = hand_data.size
    if (np.any(pose_data) == None) and (np.any(hand_data) == None):
        print("Trajectory recording empty, exiting")
        sys.exit()

    # Start playback
    if arm_connected:
        print("Going to starting pose...")
        while pose_current == None:
            pass    # Wait for pose update
        to_start_pose = interpolate_poses(pose_current, pose_data[0], 0.1/record_rate).squeeze()
        for i in range (to_start_pose.size):
            goal_pose = to_start_pose[i]
            goal_pub.publish(goal_pose)
            if key_in == Key.esc:
                break
            ros_record_rate.sleep()

    print("Playback started. Press Esc to stop.")
    for i in range (record_length):
        if arm_connected:
            goal_pose = pose_data[i]
            goal_pub.publish(goal_pose)
        if hand_connected:
            hand_setpt = hand_data[i]
            hand_setpt.header.stamp = rospy.Time.now()
            hand_pub.publish(hand_setpt)
        if key_in == Key.esc:
            break
        ros_record_rate.sleep()

    print("Playback completed.")
    