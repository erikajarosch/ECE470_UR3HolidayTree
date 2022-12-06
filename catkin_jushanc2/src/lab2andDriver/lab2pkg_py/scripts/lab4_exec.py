#!/usr/bin/env python
import copy
import time
import rospy
import os
import sys
import numpy as np
import argparse
import rospkg
import yaml
import sys
from math import pi
from lab4_header import *
from lab5_header import *
from lab4_func import *
from blob_search import *

# 20Hz
SPIN_RATE = 20 

# UR3 home location
home = [120*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

# Position for UR3 not blocking the camera
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]

# Store world coordinates of blue and yellow blocks
xw_yw_B = []
xw_yw_Y = []

# Any other global variable you want to define
# Hints: where to put the blocks?
destination_y = [[200.0,-150.0 ,32.0], [250.0, -150.0, 32.0]]
destination_b = [[300.0, -150.0,32.0], [350.0,-150.0,32.0]]





"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    global analog_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0
    analog_in_0 = msg.AIN0

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

def move_block(pub_cmd, loop_rate, start_loc, 
               end_loc, Q):
    
    global digital_in_0
    global suction_on
    global suction_off



    ### Hint: Use the Q array to map out your towers by location and "height".
    
    # print("Q",Q)
    # print(Q[start_loc][start_height])

    #Move to home position
    move_arm(pub_cmd,loop_rate, home ,1.0,1.0)
    time.sleep(0.5)

    #Move to pick up the block
    move_arm(pub_cmd,loop_rate, start_loc,1.0,1.0)
    time.sleep(0.5)

    #Turn the gripper on
    suction = gripper(pub_cmd, loop_rate, suction_on)
    rospy.sleep(1.0)

    if (current_io_0 == suction):
        gripper(pub_cmd, loop_rate, suction_off)
       
        sys.exit()
    else:
        move_arm(pub_cmd,loop_rate, home ,1.0,1.0)

        #Move to the location
        move_arm(pub_cmd, loop_rate, end_loc, 1.0,1.0)
        time.sleep(0.5)

        #Turn suction off
        gripper(pub_cmd, loop_rate, suction_off)
        time.sleep(0.5)

        move_arm(pub_cmd,loop_rate, home ,1.0,1.0)


    error = 0





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

        xw_yw_Y = blob_search(cv_image, "yellow")
        #xw_yw_Y = blob_search(cv_image, "yellow")

        
        #print("yellow", xw_yw_Y)
        #print("blue", xw_yw_B)



"""
Program run from here
"""
def main():

    global home

  # Parser
    parser = argparse.ArgumentParser(description='Please specify if using simulator or real robot')
    parser.add_argument('--simulator', type=str, default='True')
    args = parser.parse_args()

    # Initialize rospack
    rospack = rospkg.RosPack()
    # Get path to yaml
    lab2_path = rospack.get_path('lab2pkg_py')
    yamlpath = os.path.join(lab2_path, 'scripts', 'lab2_data.yaml')
    

    with open(yamlpath, 'r') as f:
        try:
            # Load the data as a dict
            data = yaml.load(f)
       
            if args.simulator.lower() == 'true':
		
                Q = data['sim_pos']
            elif args.simulator.lower() == 'false':
                Q = data['real_pos']
            else:
                print("Invalid simulator argument, enter True or False")
                sys.exit()
            
        except:
            print("YAML not found")
            sys.exit()
    

    # Initialize ROS node
    rospy.init_node('lab4node')

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
    time.sleep(3)

    new_dest = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    #if(len(sys.argv) != 5):
        #print("\n")
        #print("Invalid number of input!\n")
        #print("rosrun lab4pkg_py lab4_exec.py xWgrip yWgrip zWgrip yaw_WgripDegree \n")
    #else:
        #print("\nxWgrip: " + sys.argv[1] + ", yWgrip: " + sys.argv[2] + \
              #", zWgrip: " + sys.argv[3] + ", yaw_WgripDegree: " + sys.argv[4] + "\n")

    # User inputs the desired end location of the ornament
    xWgrip = input("Enter desired x destination as a float: ")
    yWgrip = input("Enter desired y destination as a float: ")
    zWgrip = input("Enter desired z destination as a float: ")
    yaw = input("Enter desired yaw angle in degrees: ")
    print("The desired final location is: (" + str(xWgrip) + "," + str(yWgrip) + "," + str(zWgrip) + ")\n")
    print("The desired yaw at the final location is:" + str(yaw) + "\n")
   
    

    new_dest = lab_invk(xWgrip, yWgrip, zWgrip, yaw)
    x_start = 400
    y_start = 51
    z_start = 50

    y =  0
    start_loc = lab_invk(x_start,y_start,z_start,y)
 
    #start_loc  = [2.7259748150991014, -0.44856214025722957, 0.9267454221590636, -2.048979608696731,
      #-1.5707963267948966, 1.1551784883042049]

    #print("start should be:",start_loc)
    print("start is:",lab_invk(x_start,y_start,z_start,y))
    

    vel = 4.0
    accel = 4.0

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    move_block(pub_command, loop_rate,start_loc,new_dest, Q)

    #move_arm(pub_command, loop_rate, , vel, accel)

    rospy.loginfo("Destination is reached!")



if __name__ == '__main__':
    
    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
