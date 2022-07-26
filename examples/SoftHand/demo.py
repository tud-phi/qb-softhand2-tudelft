#!/usr/bin/env python

import rospy
from math import cos
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def talker():
    pub = rospy.Publisher('/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('demo', anonymous=True)
    rate = rospy.Rate(5)
    start_time = rospy.get_time()
    while not rospy.is_shutdown():
        cmd = JointTrajectory()
        cmd.header.stamp = rospy.Time.now()
        cmd.joint_names = ['qbhand2m1_manipulation_joint', 'qbhand2m1_synergy_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [0.0, 0.33*(cos((rospy.get_time()-start_time)+3.142)+1.01)]
        pt.velocities = [0.0, 0.0]
        pt.accelerations = [0.0, 0.0]
        pt.effort = [0.0, 0.0]
        pt.time_from_start = rospy.Time.from_sec(1)
        cmd.points.append(pt)
        rospy.loginfo(cmd)
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
