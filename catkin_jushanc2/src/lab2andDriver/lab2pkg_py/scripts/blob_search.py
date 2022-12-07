#!/usr/bin/env python

from colorsys import rgb_to_hsv
import cv2
from cv2 import cornerMinEigenVal
import numpy as np



# Params for camera calibration
r = 227
c = 184
O_r = 240
O_c = 320
theta = -1.1*np.pi/180
beta = 753.57


tx= -0.228
ty = -0.137

# Function that converts image coord to world coord


def IMG2W(col, row):
    x_c = (row-O_r)/beta
    y_c = (col-O_c)/beta
    curr = np.array([x_c, y_c])
    R = [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
    x_y = np.dot(np.linalg.inv(R),(curr-np.array([tx,ty])))
   
    return (x_y[0]*1000.0, x_y[1]*1000.0)



def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    

    # Filter by Color
    params.filterByColor = False
    params.minThreshold = 1
    params.maxThreshold = 300

    # Filter by Area.
    params.filterByArea = True

    # Filter by Circularity
    params.filterByCircularity = False
    params.maxCircularity = 1

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    
    r_u = 165
    g_u = 80
    b_u = 60

 

    #lower = (8,110,60)
    #upper = (15,360,200)
    if (color == "blue"):
        lower = (75,110,30)
        upper = (105,250,255)
    if (color == "yellow"):
        lower = (20,110,30)
        upper = (40,250,255)

   


    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    

    keypoints = detector.detect(mask_image)
    

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0], keypoints[i].pt[1]))

    
    #print("blob center", blob_image_center[0])
    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, 0)

    #print("blob center", IMG2W(blob_image_center[0], blob_image_center[1]))
    
    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(
                IMG2W(blob_image_center[i][0], blob_image_center[i][1]))

    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw


