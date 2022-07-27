#!/usr/bin/env python
import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
from pynput.keyboard import Listener, Key, KeyCode

class softhand_setpt_controller():

    def __init__(self):
        self.r = rospy.Rate(100)
        # Keyboard listener for adjustment with arrow keys
        self.listener = Listener(on_press=self._on_press, suppress=False)
        self.listener.start()
        self.use_keyboard = True
        # Subscribe to SpaceMouse button input topic for button control
        self.sm_sub = rospy.Subscriber("/spacenav/joy", Joy, self.sm_joy_callback)
        self.sm_Lbutton_prev = 0
        self.sm_Rbutton_prev = 0
        self.use_sm = False
        # Subscribe to control pad input topic for trigger control
        self.sm_sub = rospy.Subscriber("/joy", Joy, self.control_pad_callback)
        self.use_control_pad = False
        self.Lbumper_prev = 0
        self.Rbumper_prev = 0
        # Setpoint publisher
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
        # State flags
        self.synergy_close = False  # Toggle open/closed with sm button
        self.manip_pinch = False    # Toggle pinch grip with sm button
        self.synergy_lock = False   # Lock synergy with control pad input
        self.manip_lock = False     # Lock manipulation with control pad input

    def _on_press(self, key):
        # Control input toggles
        if key == KeyCode.from_char('1'):
            self.use_keyboard = not self.use_keyboard
            if self.use_keyboard: 
                print("\nSofthand keyboard input on")
                if self.use_control_pad:
                    self.use_control_pad = False
                    print("Softhand control pad input off")
            else: print("\nSofthand keyboard input off")
        if key == KeyCode.from_char('3'):
            self.use_sm = not self.use_sm
            if self.use_sm: print("\nSofthand 3D mouse input on")
            else: print("\nSofthand 3D mouse input off")
        if key == KeyCode.from_char('4'):
            self.use_control_pad = not self.use_control_pad
            if self.use_control_pad: 
                print("\nSofthand control pag input on")
                if self.use_keyboard:
                    self.use_keyboard = False
                    print("Softhand keyboard input off")
            else: print("\nSofthand control pad input off")

        if self.use_keyboard == True:
            if key == Key.up:
                self.synergy_setpt += 0.01
            elif key == Key.down:
                self.synergy_setpt -= 0.01
            elif key == Key.left:
                self.manip_setpt += 0.01
            elif key == Key.right:
                self.manip_setpt -= 0.01
            elif key == Key.page_up:
                self.synergy_setpt = 1.0
            elif key == Key.page_down:
                self.synergy_setpt = 0.0
            elif key == KeyCode.from_char(','):
                self.manip_setpt = 1.0
            elif key == KeyCode.from_char('.'):
                self.manip_setpt = 0.0
            self.send_command()

    def sm_joy_callback(self, sm_input):
        if self.use_sm:
            if (sm_input.buttons[0] == 1) and (self.sm_Lbutton_prev != 1):
                self.synergy_close = not self.synergy_close
                if self.synergy_close:
                    self.synergy_setpt = 1.0
                else:
                    self.synergy_setpt = 0.0
            elif (sm_input.buttons[1] == 1) and (self.sm_Rbutton_prev != 1):
                self.manip_pinch = not self.manip_pinch
                if self.manip_pinch:
                    self.manip_setpt = 0.0
                else:
                    self.manip_setpt = 1.0
            self.send_command()
            self.sm_Lbutton_prev = sm_input.buttons[0]
            self.sm_Rbutton_prev = sm_input.buttons[1]

    def control_pad_callback(self, joy_input):
        if self.use_control_pad:
            if joy_input.buttons[5] and self.Rbumper_prev != 1:
                self.synergy_lock = not self.synergy_lock
            if joy_input.buttons[4] and self.Lbumper_prev != 1:
                self.manip_lock = not self.manip_lock
            if not self.synergy_lock:
                self.synergy_setpt = (joy_input.axes[5]-1)/-2.0
            if not self.manip_lock:
                self.manip_setpt = (joy_input.axes[2]-1)/-2.0
            self.send_command()
            self.Lbumper_prev = joy_input.buttons[4]
            self.Rbumper_prev = joy_input.buttons[5]
            
    def send_command(self):
        self.synergy_setpt = np.clip(self.synergy_setpt, 0.0, 1.0)
        self.manip_setpt = np.clip(self.manip_setpt, 0.0, 1.0)
        self.setpt.positions = [self.manip_setpt, self.synergy_setpt]
        self.setpt_cmd.header.stamp = rospy.Time.now()
        self.setpt_cmd.points.clear()
        self.setpt_cmd.points.append(self.setpt)
