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
        interp_pose.header.seq = i
        interp_pose.header.stamp = rospy.Time.now()
        interp_pose.header.frame_id = "interpolated" # not really a frame... just to indicate which points are interpolated

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

def interpolate_joints(config_start, config_end, interp_ang):
    start_pos = np.array(config_start.position)
    end_pos = np.array(config_end.position)
    step_num = math.floor(np.max(np.abs(end_pos-start_pos))/interp_ang)
    interp_pos = np.linspace(start_pos, end_pos, step_num)

    interpolated_traj = config_start

    for i in range(step_num):
        interp_joints = JointState()
        interp_joints.name = config_start.name
        interp_joints.header.seq = i
        interp_joints.header.stamp = rospy.Time.now()
        interp_joints.header.frame_id = "interpolated"

        interp_joints.position = interp_pos[i]

        interpolated_traj = np.c_[interpolated_traj, interp_joints]

    return interpolated_traj

def interpolate_softhand(setpt_start, setpt_end, interp_dist):
    start_setpt = np.array(setpt_start.points.positions)
    end_setpt = np.array(setpt_end.points.positions)
    step_num = math.floor(np.max(np.abs(end_setpt-start_setpt))/interp_dist)
    interp_setpt = np.linspace(start_setpt, end_setpt, step_num)

    interpolated_traj = setpt_start

    for i in range(step_num):
        interp_setpt = JointTrajectory()
        interp_setpt.joint_names = interp_setpt.joint_names
        interp_setpt.header.seq = i
        interp_setpt.header.stamp = rospy.Time.now()
        interp_setpt.header.frame_id = "interpolated"

        interp_setpt.points.positions = interp_setpt[i]
        interp_setpt.points.velocities = [0.0, 0.0]
        interp_setpt.accelerations = [0.0, 0.0]
        interp_setpt.effort = [0.0, 0.0]
        interp_setpt.time_from_start = rospy.Time.from_sec(0.25) # TODO - eliminate manual delay?

        interpolated_traj = np.c_[interpolated_traj, interp_setpt]

    return interpolated_traj

#%%
if __name__ == '__main__':

    rospy.init_node('traj_recorder', anonymous=True)

    key_listener = keyboard.Listener(on_press=_on_press, suppress=False)
    key_listener.start()
    key_in = None

    arm_connected = True if "/panda_controller_spawner" in rosnode.get_node_names() else False # TODO - check node name for real connection
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
                # Insert interpolations into trajectories # TODO - add padding to make sure all interpolated trajectories same length
                if arm_connected:
                    interpolated_poses = interpolate_poses(pose_at_pause, pose_current, 0.1/record_rate) # 0.1m/s
                    pose_trajectory = np.c_[pose_trajectory, interpolated_poses]
                    interpolated_joints = interpolate_joints(joints_at_pause, joint_state_current, np.pi/4/record_rate) # 45deg/s
                    joint_trajectory = np.c_[joint_trajectory, interpolated_joints]
                if hand_connected:
                    interpolated_setpts = interpolate_softhand(setpts_at_pause, hand_command_current, 0.5/record_rate) # close halfway/s 
                    hand_trajectory = np.c_[hand_trajectory, interpolated_setpts]

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
