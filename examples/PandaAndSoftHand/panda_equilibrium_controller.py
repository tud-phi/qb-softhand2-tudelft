#!/usr/bin/env python
import rospy
import numpy as np
import quaternion # pip install numpy-quaternion
from geometry_msgs.msg import PoseStamped, Twist
import dynamic_reconfigure.client
from pynput import keyboard, mouse
from pynput.keyboard import KeyCode, Key

class panda_equilibrium_controller():

    def __init__(self):
        self.r = rospy.Rate(100)
        # Create listener using Pynput to handle keyboard input
        self.key_listener = keyboard.Listener(on_press=self._on_press, suppress=False)
        self.key_listener.start()
        self.use_keyboard = False
        self.suspend_key_input = False
        # Create listener for mouse input
        self.prev_mouse_pos = None
        self.mouse_listener = mouse.Listener(on_move=self._on_mouse_move, on_scroll=self._on_mouse_scroll, suppress=False)
        self.mouse_listener.start()
        self.use_mouse = False
        # Subscribe to SpaceMouse input topic
        self.sm_sub = rospy.Subscriber("/spacenav/twist", Twist, self.sm_input_callback)
        self.use_sm = False
        # Goal pose subcriber and publisher
        self.K_pos = 600
        self.K_ori = 30
        self.K_ns = 10
        self.curr_pos = None
        self.curr_ori = None
        self.pos_sub = rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback)
        self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
        self.goal = PoseStamped()

    def _on_press(self, key):
        if self.suspend_key_input:
            return

        # Control input toggles
        if key == KeyCode.from_char('1'):
            self.use_keyboard = not self.use_keyboard
            if self.use_keyboard: print("\nPanda keyboard input on")
            else: print("\nPanda keyboard input off")
        if key == KeyCode.from_char('2'):
            self.use_mouse = not self.use_mouse
            if self.use_mouse: print("\nPanda mouse input on")
            else: print("\nPanda mouse input off")
        if key == KeyCode.from_char('3'):
            self.use_sm = not self.use_sm
            if self.use_sm: print("\nPanda 3D mouse input on")
            else: print("\nPanda 3D mouse input off")

        # Keyboard control input
        if self.use_keyboard == True:
            rot_quat = None
            # Nullspace stiffness
            if key == KeyCode.from_char('n'):
                self.set_stiffness(self.K_pos, self.K_pos, self.K_pos ,self.K_ori,self.K_ori,self.K_ori, 1.0)
            elif key == KeyCode.from_char('j'):
                self.set_stiffness(self.K_pos, self.K_pos, self.K_pos ,self.K_ori,self.K_ori,self.K_ori, 0.0)
            # Change goal position
            elif key == KeyCode.from_char('w'):
                self.goal.pose.position.x += 0.01
            elif key == KeyCode.from_char('s'):
                self.goal.pose.position.x -= 0.01
            elif key == KeyCode.from_char('a'):
                self.goal.pose.position.y += 0.01
            elif key == KeyCode.from_char('d'):
                self.goal.pose.position.y -= 0.01
            elif key == KeyCode.from_char('q'):
                self.goal.pose.position.z += 0.01
            elif key == KeyCode.from_char('z'):
                self.goal.pose.position.z -= 0.01
            # Change goal orientation
            elif key == KeyCode.from_char('h'): # pos rot on x axis
                rot_quat = quaternion.from_rotation_vector(np.array([np.pi/40, 0, 0]))
            elif key == KeyCode.from_char('f'): # neg rot on x axis
                rot_quat = quaternion.from_rotation_vector(np.array([-np.pi/40, 0, 0]))
            elif key == KeyCode.from_char('t'): # pos rot on y axis
                rot_quat = quaternion.from_rotation_vector(np.array([0, np.pi/40, 0]))
            elif key == KeyCode.from_char('g'): # neg rot on y axis
                rot_quat = quaternion.from_rotation_vector(np.array([0, -np.pi/40, 0]))
            elif key == KeyCode.from_char('r'): # pos rot on z axis
                rot_quat = quaternion.from_rotation_vector(np.array([0, 0, np.pi/40]))
            elif key == KeyCode.from_char('y'): # neg rot on z axis
                rot_quat = quaternion.from_rotation_vector(np.array([0, 0, -np.pi/40]))
            if rot_quat is not None:
                orient = np.quaternion(self.goal.pose.orientation.w, self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z)
                new_orient = rot_quat*orient
                new_orient = new_orient/np.linalg.norm(quaternion.as_float_array(new_orient)) # Re normalise
                self.goal.pose.orientation.w = new_orient.w
                self.goal.pose.orientation.x = new_orient.x
                self.goal.pose.orientation.y = new_orient.y
                self.goal.pose.orientation.z = new_orient.z

    def _on_mouse_move(self, x, y):
        if self.use_mouse:
            # Note: '+ve mouse X' -> '-ve robot Y' etc
            dx = -(y-self.prev_mouse_pos[1])
            dy = -(x-self.prev_mouse_pos[0])
            dx = np.clip(dx, -3, 3)
            dy = np.clip(dy, -3, 3)
            self.goal.pose.position.x += dx*0.001
            self.goal.pose.position.y += dy*0.001

    def _on_mouse_scroll(self, x, y, dx, dy):
        if self.use_mouse:
            # dy corresponds to scroll wheel (dx would be thumb scroll)
            self.goal.pose.position.z += dy*0.01

    def sm_input_callback(self, sm_twist):
        if self.use_sm:
            rot_quat = None
            # Change goal position
            self.goal.pose.position.x += 0.002*sm_twist.linear.x
            self.goal.pose.position.y += 0.002*sm_twist.linear.y
            self.goal.pose.position.z += 0.002*sm_twist.linear.z
            # Change goal orientation
            rot_quat = quaternion.from_rotation_vector(0.005*np.array([sm_twist.angular.x, sm_twist.angular.y, sm_twist.angular.z]))
            orient = np.quaternion(self.goal.pose.orientation.w, self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z)
            new_orient = rot_quat*orient
            new_orient = new_orient/np.linalg.norm(quaternion.as_float_array(new_orient)) # Re normalise
            self.goal.pose.orientation.w = new_orient.w
            self.goal.pose.orientation.x = new_orient.x
            self.goal.pose.orientation.y = new_orient.y
            self.goal.pose.orientation.z = new_orient.z

    def ee_pos_callback(self, data):
        self.curr_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.curr_ori = np.array([data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])

    def set_stiffness(self, k_t1, k_t2, k_t3,k_r1,k_r2,k_r3, k_ns):
        set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        set_K.update_configuration({"translational_stiffness_X": k_t1})
        set_K.update_configuration({"translational_stiffness_Y": k_t2})
        set_K.update_configuration({"translational_stiffness_Z": k_t3})        
        set_K.update_configuration({"rotational_stiffness_X": k_r1}) 
        set_K.update_configuration({"rotational_stiffness_Y": k_r2}) 
        set_K.update_configuration({"rotational_stiffness_Z": k_r3})
        set_K.update_configuration({"nullspace_stiffness": k_ns})   