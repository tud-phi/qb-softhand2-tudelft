#%%
#!/usr/bin/env python
import rospy
import time
import numpy as np
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from pynput.keyboard import Listener, Key

class softhand_setpt_controller():

    def __init__(self):
        self.r=rospy.Rate(100)
        self.end=False
        self.listener = Listener(on_press=self._on_press, suppress=False)
        self.listener.start()

        self.setpt_pub = rospy.Publisher('/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command', JointTrajectory, queue_size=0)
        self.synergy_setpt = 0.0
        self.manip_setpt = 0.5
        self.setpt = JointTrajectoryPoint()
        self.setpt.positions = [self.manip_setpt, self.synergy_setpt]
        self.setpt.velocities = [0.0, 0.0]
        self.setpt.accelerations = [0.0, 0.0]
        self.setpt.effort = [0.0, 0.0]
        self.setpt.time_from_start = rospy.Time.from_sec(0.25)	# TODO - elminate manual delay?
        self.setpt_cmd = JointTrajectory()

    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == Key.esc:
            self.end = True
        elif key == Key.up:
            self.synergy_setpt += 0.01
        elif key == Key.down:
            self.synergy_setpt -= 0.01
        elif key == Key.left:
            self.manip_setpt += 0.01
        elif key == Key.right:
            self.manip_setpt -= 0.01
        self.synergy_setpt = np.clip(self.synergy_setpt, 0.0, 1.0)
        self.manip_setpt = np.clip(self.manip_setpt, 0.0, 1.0)
        self.setpt.positions = [self.manip_setpt, self.synergy_setpt]
        self.setpt_cmd.header.stamp = rospy.Time.now()
        self.setpt_cmd.points.clear()
        self.setpt_cmd.points.append(self.setpt)

        # print("\tSynergy: ", self.setpt.positions[1], "\t\t\t\tManipulation: ", self.setpt.positions[0])
        print(f'\tSynergy: {self.setpt.positions[1]:.2f}\tManipulation: {self.setpt.positions[0]:.2f}')


#%%
if __name__ == '__main__':
    rospy.init_node('keyboard_control', anonymous=True)
    start_time = time.time()

#%%    
    # Instantiate softhand setpt controller 
    SHC = softhand_setpt_controller()
    # Set initial setpt
    SHC.setpt_cmd.header.stamp = rospy.Time.now()
    SHC.setpt_cmd.joint_names = ['qbhand2m1_manipulation_joint', 'qbhand2m1_synergy_joint']
    SHC.setpt_cmd.points.append(SHC.setpt)
    
    # Print controls
    print("Keyboard controls:\n" \
        + "\tup/down:\t hand synergy +/-\n" \
        + "\tleft/right:\t hand manipulation +/-\n")

    while SHC.end != True:
        SHC.setpt_pub.publish(SHC.setpt_cmd)
        SHC.r.sleep()

#%%
