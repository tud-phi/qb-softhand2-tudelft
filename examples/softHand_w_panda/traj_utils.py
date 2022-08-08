#%%
#!/usr/bin/env python
import numpy as np
import math
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Adapted from Human Friendly Controllers - Learning_from_demonstration.py - go_to_pose()
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
    step_num = np.max([1,step_num])

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
    step_num = np.max([1,step_num])
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
    start_setpt = np.array(setpt_start.points[0].positions)
    end_setpt = np.array(setpt_end.points[0].positions)
    step_num = math.floor(np.max(np.abs(end_setpt-start_setpt))/interp_dist)
    step_num = np.max([1,step_num])
    interp_setpts = np.linspace(start_setpt, end_setpt, step_num)

    interpolated_traj = setpt_start

    for i in range(step_num):
        interp_setpt = JointTrajectory()
        interp_setpt.joint_names = interp_setpt.joint_names
        interp_setpt.header.seq = i
        interp_setpt.header.stamp = rospy.Time.now()
        interp_setpt.header.frame_id = "interpolated"

        interp_setptpt = JointTrajectoryPoint()
        interp_setptpt.positions = interp_setpts[i]
        interp_setptpt.velocities = [0.0, 0.0]
        interp_setptpt.accelerations = [0.0, 0.0]
        interp_setptpt.effort = [0.0, 0.0]
        interp_setptpt.time_from_start = rospy.Time.from_sec(0.25) # TODO - eliminate manual delay?

        interp_setpt.points.append(interp_setptpt)

        interpolated_traj = np.c_[interpolated_traj, interp_setpt]

    return interpolated_traj