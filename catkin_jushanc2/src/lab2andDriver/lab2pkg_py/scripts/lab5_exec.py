#!/usr/bin/env python

import sys
import copy
import time
import rospy
import yaml

import numpy as np
from lab5_header import *
from lab4_func import *
from blob_search import *



# ========================= Student's code starts here =========================

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# Store world coordinates of blue and yellow blocks
xw_yw_B = []
xw_yw_Y = []

# Any other global variable you want to define
# Hints: where to put the blocks?
destination_y = [[200.0,-150.0 ,32.0], [250.0, -150.0, 32.0]]
destination_b = [[300.0, -150.0,32.0], [350.0,-150.0,32.0]]

# ========================= Student's code ends here ===========================

################ Pre-defined parameters and functions no need to change below ################

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]


# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

image_shape_define = False


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            #rospy.loginfo("Goal is reached!")
            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            #rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

################ Pre-defined parameters and functions no need to change above ################


def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel, h):

    """
    start_xw_yw_zw: where to pick up a block in global coordinates
    target_xw_yw_zw: where to place the block in global coordinates

    hint: you will use lab_invk(), gripper(), move_arm() functions to
    pick and place a block

    """
    # ========================= Student's code starts here =========================

    global go_away
    global digital_in_0
    global suction_on
    global suction_off

    start_x = start_xw_yw_zw[0]*1000
    start_y = start_xw_yw_zw[1]*1000

    end_x = target_xw_yw_zw[0]
    end_y = target_xw_yw_zw[1]
    ### Hint: Use the Q array to map out your towers by location and "height".
    start_block_pre = lab_invk(start_x, start_y, 34.0, 0)
    end_block_pre = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    print(start_y-50)
    if (h == 185.0):
	end_block_pre = lab_invk(end_x+3, end_y-10.0, h, 0.0)
    elif (h == 240.0):
	end_block_pre = lab_invk(end_x+40, end_y+8, h, 0.0)
    elif (h == 280.0):
	end_block_pre = lab_invk(end_x+40, end_y+8, h, 0.0)
    start_block = lab_invk(start_xw_yw_zw[0]-20, start_xw_yw_zw[1]-40, 34.0, 0)
    end_block = lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], 34.0, 0)
    mid_pos = lab_invk(end_block_pre[0], end_block_pre[1], h, 0)
    
    
    #Move to get ready 
    move_arm(pub_cmd,loop_rate, start_block_pre,vel,accel)
    #time.sleep(0.5)
    # move to pick up the block
    #move_arm(pub_cmd,loop_rate, start_block,vel,accel)
    time.sleep(0.5)
    print("hi")
    #Turn the gripper on
    suction = gripper(pub_cmd, loop_rate, suction_on)
    rospy.sleep(1.0)

    if (digital_in_0 == suction):
        gripper(pub_cmd, loop_rate, suction_off)

        move_arm(pub_cmd,loop_rate, go_away ,vel,accel)
        print("Block is missing")
        
    else:
    

        #Move to the location
        move_arm(pub_cmd,loop_rate, mid_pos,vel,accel)
        move_arm(pub_cmd, loop_rate, end_block_pre, vel, accel)
        time.sleep(0.5)
        #move_arm(pub_cmd, loop_rate, end_block, vel, accel)
        #time.sleep(0.5)

        #Turn suction off
        gripper(pub_cmd, loop_rate, suction_off)
        time.sleep(0.5)

        #move_arm(pub_cmd,loop_rate, go_away ,vel,accel)


    error = 0

    # ========================= Student's code ends here ===========================

    return error


class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
            print("ROS is shutdown!")


    def image_callback(self, data):

        global xw_yw_B # store found green blocks in this list
        global xw_yw_Y # store found yellow blocks in this list

        try:
          # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(raw_image, -1)
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        # You will need to call blob_search() function to find centers of green blocks
        # and yellow blocks, and store the centers in xw_yw_G & xw_yw_Y respectively.

        # If no blocks are found for a particular color, you can return an empty list,
        # to xw_yw_G or xw_yw_Y.

        # Remember, xw_yw_G & xw_yw_Y are in global coordinates, which means you will
        # do coordinate transformation in the blob_search() function, namely, from
        # the image frame to the global world frame.

        xw_yw_B = blob_search(cv_image, "yellow")
        #xw_yw_Y = blob_search(cv_image, "yellow")

        
        #print("yellow", xw_yw_Y)
        #print("blue", xw_yw_B)

def get_block_pos(path):
	with open(path, 'r') as f:
            # Load the data as a dict
            data = yaml.load(f)
            # Load block position
            block_xy_pos = data['block_xy_pos']
            return block_xy_pos



        

"""
Program run from here
"""
def main():

    global go_away
    global xw_yw_B
    global xw_yw_Y

    # global variable1
    # global variable2
    

    # Initialize ROS node
    rospy.init_node('lab5node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)
    
    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    vel = 3
    accel = 3
    #move_arm(pub_command, loop_rate, go_away, vel, accel)
    

    ic = ImageConverter(SPIN_RATE)
    time.sleep(5)

    # ========================= Student's code starts here =========================

    """
    Hints: use the found xw_yw_G, xw_yw_Y to move the blocks correspondingly. You will
    need to call move_block(pub_command, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel)
    """
    

    #Y1 = xw_yw_Y[0]
    #Y2 = xw_yw_Y[1]
    B1 = xw_yw_B[0]
    
    #Assign arbitrary positions to the ledges (from top to bottom)
    ledge_1 = xw_yw_B[3]
    ledge_2 = xw_yw_B[4]
    ledge_3 = xw_yw_B[1]
    ledge_4 = xw_yw_B[5]
    ledge_5 = xw_yw_B[2]
    ledge_6 = xw_yw_B[0]
    print(ledge_1)
    
    #B2 = xw_yw_B[1]

    #Block Positions 
    path = "/home/ur3/catkin_jushanc2/src/lab2andDriver/lab2pkg_py/scripts/lab2_data.yaml"
    blocks = get_block_pos(path)
    print(blocks)
    
    green_1 = blocks[0][0]
    green_2 = blocks[1][0]

    blue_1 = blocks[0][1]
    blue_2 = blocks[1][1]
    
    red_1 = blocks[0][2]
    red_2 = blocks[1][2]

    

    h_1 = 185.0
    h_2 = 240.0
    h_3 = 280.0


    #move_block(pub_command,loop_rate, red_1, destination_y[0], vel,accel, 50.0)
    #move_arm(pub_command,loop_rate, go_away,vel,accel)
    #move_block(pub_command,loop_rate, red_2,destination_y[1],vel,accel, 50.0)
    #move_block(pub_command,loop_rate, green_1,destination_b[0],vel,accel,50.0)
    #move_block(pub_command,loop_rate, go_away,vel,accel)
    #move_block(pub_command,loop_rate, green_2,destination_b[1],vel,accel,50.0)
    #move_block(pub_command,loop_rate, go_away,vel,accel)
    #move_block(pub_command,loop_rate, blue_1,red_1,vel,accel,50.0)
    #move_block(pub_command,loop_rate, go_away,vel,accel)
    #move_arm(pub_command,loop_rate, blue_2,red_2,vel,accel,50.0)
    #move_block(pub_command,loop_rate, go_away,vel,accel)

    #user_color = input("Which color ornament do you want to place <red, blue, green>:" )
    #user_block = input("Which ornament do you want to move <1,2,3 from left side>: ")
    
    #pick_up
    
    
    #move_block(pub_command, loop_rate, Y1, destination_b[0], vel, accel)
    #move_block(pub_command, loop_rate, Y2, destination_b[1], vel, accel)
    
    print(ledge_6)
    #move_block(pub_command, loop_rate, red_1, ledge_6, vel, accel, h_1)
    #move_block(pub_command, loop_rate, red_2, ledge_5, vel, accel, h_1)
    #move_block(pub_command, loop_rate, blue_1, ledge_2, vel, accel, h_2)
    #move_block(pub_command, loop_rate, green_2, ledge_3, vel, accel, h_2)
    move_block(pub_command, loop_rate, green_2, ledge_1, vel, accel, h_3)
    
    
    #move_block(pub_command, loop_rate, B2, destination_y[1], vel, accel)


   

    # ========================= Student's code ends here ===========================

    move_arm(pub_command, loop_rate, go_away, vel, accel)
    rospy.loginfo("Task Completed!")
    print("Use Ctrl+C to exit program")
    rospy.spin()

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass


