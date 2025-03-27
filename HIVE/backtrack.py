import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import imutils
import time
import math
from math import isnan
import sys

# add directories and import custom "libraries"
sys.path.append('D:/all my files/documents/uvm/5_masters/hive/github_directory/HIVE/vision/')
import vision_continuous as vision

sys.path.append('D:/all my files/documents/uvm/5_masters/hive/github_directory/HIVE/motion/python/send_serial_motor_control/')
import send_actuation_msg as messenger
#
sys.path.append('D:/all my files/documents/uvm/5_masters/hive/github_directory/HIVE/mapping/')
from mapping import px2rad


####################################################
#               vehicle parameters                 #
####################################################
# vehicle measurements
wheel_base_m = 0.158    # [m]
tire_radius_m = 0.022   # [m]

# set camera specs
wp = 640
theta_fov_depth = math.radians(81)
cam_loc = [-25,105] # sideways,forward displacement of camera (right +,fwd +)

print('*************************************')
print('Vehicle and Controller Specifications')
print('*************************************')
print('')
print('wheel base [m]:    ' + str(wheel_base_m))
print('tire radius [m]:   ' + str(tire_radius_m))

print('***************************')
print('Backtrack.')
print('25 MAR 20')
print('***************************')

'''
cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
'''
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

print("Time Elapsed [s],"
    "Dist [mm],Head [rad],"
    "x [mm],y [mm],"
    "s [m/s],omega [rad/s],"
    "omega_l [rad/s],omega_r [rad/s]")

head_rad = 0
dist_mm = 0
s = 0
omega = 0
omega_l = 0
omega_r = 0

try:
    start = time.time()

    time_now = start
    head_rad = 1
    x = 1
    y = 1

    time_old = 0
    head_rad_old = 0
    x_old = 0
    y_old = 0

    while True:
        time_now = time.time()

        # get images (depth and color) from camera
        image_pair = vision.get_aligned_frame(pipeline)
        depth_image = image_pair[1]
        color_image = image_pair[0]

        # detect markers in images
        markers_list = vision.detect_aruco(image_pair,
                                           visualize=False, camera_location=cam_loc)
            # id = markers[0][0],   id,       [-]
            # x = markers[0][1][0], y,        [px]
            # y = markers[0][1][1], x,        [px]
            # d = markers[0][2],    distance, [mm]
            # h = markers[0][3]))   heading,  [rad]

        if markers_list: # check to see if markers contains any elements (not empty)
            # convert markers from list to array
            markers = np.asarray(markers_list)

            # if multiple markers detected (e.g. marker on back of leader and side of leader)
            # then take the average and min position of them
            markers_avg = np.mean(markers, axis=0)
            markers_min = np.min(markers, axis=0)

            # extract distance and heading detected in image
            # taking MINIMUM distance to avoid lurching when additional markers are detected
            # taking AVERAGE heading to help steer... may need to change this to min as well? needs testing
            dist_mm = markers_min[3]    # distance to aruco [mm]
            head_rad = markers_avg[4] # heading to aruco  [rad]

            ############################
            #       back-track         #
            ############################
            dt = time_now - time_old

            x = dist_mm*np.sin(head_rad)
            y = -dist_mm*np.cos(head_rad)

            omega = -(head_rad-head_rad_old)/dt
            s = np.sqrt((x-x_old)**2+(y-y_old)**2)/1000

            omega_l = (s-omega*wheel_base_m/2)/tire_radius_m
            omega_r = (s+omega*wheel_base_m/2)/tire_radius_m

            time_old = time_now
            head_rad_old = head_rad
            x_old = x
            y_old = y
        '''
        #######################################################################################
        # put together view window #
        #######################################################################################

        [color_unaligned,_] = vision.get_curr_frame(pipeline)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        #################
        #  good alphas  #
        # ------------- #
        # 0.08 standard (all looks blue)
        # 0.3  good depth of field
        #################
        scaled_depth=cv2.convertScaleAbs(depth_image, alpha=0.08)
        depth_colormap = cv2.applyColorMap(scaled_depth, cv2.COLORMAP_JET)

        ###########
        # markers #
        ###########
        if markers_list:
            for mark in markers:
                for img in [color_image, depth_colormap]:
                    cv2.circle(img, (int(mark[1]), int(mark[2])), 4, (0,0,255), -1)
                    cv2.line(img, (int(mark[5]), int(mark[6])), (int(mark[7]), int(mark[8])), (0,255,0), 2)
                    cv2.line(img, (int(mark[7]), int(mark[8])), (int(mark[9]), int(mark[10])), (0,255,0), 2)
                    cv2.line(img, (int(mark[9]), int(mark[10])), (int(mark[11]), int(mark[12])), (0,255,0), 2)
                    cv2.line(img, (int(mark[11]), int(mark[12])), (int(mark[5]), int(mark[6])), (0,255,0), 2)

        # Stack images horizontally
        images = np.hstack((color_unaligned,color_image, depth_colormap))

        # Show images
        cv2.imshow('RealSense', images)

        k = cv2.waitKey(1) & 0xFF # escape key to stop
        if k == 27:
            break
        '''

        end = time.time()
        elapsed = end-start
        #######################################################################################
        # print outputs #
        #######################################################################################
        print("{t:.5f},{d:6.1f},{h:8.4f},"
            "{x:8.2f},{y:8.2f},"
            "{s:7.4f},{o:7.3f}, "
            "{o_l:7.3f}, {o_r:7.3f}".format(
            t = elapsed,
            d = dist_mm,
            h = head_rad,#*180/3.1415,
            x = x,
            y = y,
            s = s,
            o = omega,
            o_l = omega_l,
            o_r = omega_r))

finally:
    # Stop streaming
    pipeline.stop()
    print("Stream stopped.")
