#%%
#!/usr/bin/env python
import rospy
import rosnode
import panda_equilibrium_controller
import softhand_setpt_controller
from pynput import keyboard, mouse

global key_in

def _on_press(key):
    global key_in
    key_in = key

#%%
if __name__ == '__main__':
    rospy.init_node('arm_hand_key_control', anonymous=True)

#%%    

    key_listener = keyboard.Listener(on_press=_on_press, suppress=False)
    key_listener.start()
    key_in = None

    if "/franka_control" in rosnode.get_node_names():
        hand_only = False
    else:
        hand_only = True

    # Initialise PEC controller only if Panda connected, otherwise will block while waiting for dynamic reconfigure
    if not hand_only:
        mouse_state = mouse.Controller()

        PEC = panda_equilibrium_controller.panda_equilibrium_controller()
        PEC.set_stiffness(PEC.K_pos, PEC.K_pos, PEC.K_pos ,PEC.K_ori,PEC.K_ori,PEC.K_ori, 0.0)
        PEC.prev_mouse_pos = mouse_state.position
        # Set initial goal    
        PEC.goal.header.seq = 1
        PEC.goal.header.stamp = rospy.Time.now()
        PEC.goal.header.frame_id = ""
        PEC.goal.pose.position.x = PEC.curr_pos[0]
        PEC.goal.pose.position.y = PEC.curr_pos[1]
        PEC.goal.pose.position.z = PEC.curr_pos[2]
        PEC.goal.pose.orientation.x = PEC.curr_ori[1]
        PEC.goal.pose.orientation.y = PEC.curr_ori[2]
        PEC.goal.pose.orientation.z = PEC.curr_ori[3]
        PEC.goal.pose.orientation.w = PEC.curr_ori[0]

    # Instantiate softhand setpt controller 
    SHC = softhand_setpt_controller.softhand_setpt_controller()
    # Set initial setpt
    SHC.setpt_cmd.header.stamp = rospy.Time.now()
    SHC.setpt_cmd.joint_names = ['qbhand2m1_manipulation_joint', 'qbhand2m1_synergy_joint']
    SHC.setpt_cmd.points.append(SHC.setpt)
    
    print("\nUse number keys to toggle control inputs on/off:\n" \
        + "\t1. Keyboard\t2. Mouse\t3. 3D mouse\t4. Control pad\n\n" \
        + "Keyboard controls:\n" \
        + "\tPanda:\n" \
        + "\tW/S:\t\t x-position +/-\n" \
        + "\tA/D:\t\t y-position +/-\n" \
        + "\tQ/Z:\t\t z-position +/-\n" \
        + "\tH/F:\t\t x-orientation +/-\n" \
        + "\tT/G:\t\t y-orientation +/-\n" \
        + "\tR/Y:\t\t z-orientation +/-\n" \
        + "\tN/J:\t\t Nullspace stiffness on/off\n\n" \
        + "\tSoftHand:\n" \
        + "\tup/down:\t synergy +/-\n" \
        + "\tleft/right:\t manipulation +/-\n" \
        + "\tPgUp/PgDown:\t toggles synergy fully open/closed\n" \
        + "\t,/.:\t\t toggles manipulation fully left/right\n" )
    print("Mouse controls:\n" \
        + "\tMovement:\t X/Y position\n" \
        + "\tScroll wheel: \t Z position\n" )
    print("3D mouse controls:\n" \
        + "\tJoystick:\t Panda goal pose adjustment\n" \
        + "\tL button:\t Toggle SoftHand synergy fully open/closed\n" \
        + "\tR button:\t Toggle SoftHand manipulation fully left/right\n" )
    print("Control pad controls (use analog trigger mode):\n" \
        + "\tLeft trigger:\t Control SoftHand manipulation\n" \
        + "\tRight trigger:\t Control SoftHand synergy\n" \
        + "\tLeft bumper:\t Lock SoftHand manipulation\n" \
        + "\tRight bumper:\t Lock SoftHand synergy\n" )
    print("Esc: Quit\n")

    while not (key_in == keyboard.Key.esc):
        if not hand_only:
            PEC.goal_pub.publish(PEC.goal)
            PEC.r.sleep() 
            PEC.prev_mouse_pos = mouse_state.position
        SHC.setpt_pub.publish(SHC.setpt_cmd)
        SHC.r.sleep()
        

#%%
