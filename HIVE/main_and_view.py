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
#
# sys.path.append('D:/all my files/documents/uvm/5_masters/hive/github_directory/HIVE//')
# import  as
#
# sys.path.append('D:/all my files/documents/uvm/5_masters/hive/github_directory/HIVE//')
# import  as

#############################################################################################

####################################################
#               vehicle parameters                 #
####################################################
# vehicle measurements
wheel_base_m = 0.158    # [m]
tire_radius_m = 0.022   # [m]

# max speed
s_max_mps = 0.66        # [m/s]   max speed of vehicle
omega_max = 4           # [rad/s] max omega of vehicle
omega_motor_max = 35    # [rad/s] max omega of motors

# proportional controllers
kp_speed = 0.001 # 0.002 good in isolation

#kp_heading = .015 # 0.01, 0.015 good in isolation [with pixel heading]
kp_heading = 6 # 10 good in isolation (with radian heading)

#kp_speed = 0
kp_heading = 0

# feedback initialize
dist_mm =  0
head_px =  0
head_rad = 0
markers = []

follow_dist_mm = 500 # nose: 250 // center: 300,
follow_dist_mm = 300

# set camera specs
wp = 640
theta_fov_depth = math.radians(87)
cam_loc = [-25,105] # sideways,forward displacement of camera (right +,fwd +)

####################################################
#               start serial comm                  #
####################################################
ser = messenger.initialize_com(38400);
print('waiting for connection....')
time.sleep(1)
print('*************************************')
print('Vehicle and Controller Specifications')
print('*************************************')
print('')
print('wheel base [m]:    ' + str(wheel_base_m))
print('tire radius [m]:   ' + str(tire_radius_m))
print('')
print('max speed [m/s]:   ' + str(s_max_mps))
print('omega max [rad/s]: ' + str(omega_max))
print('')
print('camera offset (fwd,horiz) [mm]: (' + str(cam_loc[0]) + ', ' + str(cam_loc[1]) + ')')
print('')
print('follow dist [mm]:  ' + str(follow_dist_mm))
print('')
print('kp_heading:   ' + str(kp_heading))
print('kp_speed:     ' + str(kp_speed))
print('')
print('***************************')

print('Heading gain test. Long step (1000mm)')
print('24 NOV 21')
print('***************************')

#############################################################################################


cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

print("Time Elapsed [s], Dist [mm], Head [rad], s_des [m/s], omega_des [rad/s], omega_l_des [rad/s], omega_r_des [rad/s]")
        # d = dist_mm,
        # h = head_rad,

        # s = s_des,
        # o_d = omega_des,

        # o_l = omega_l_des,
        # o_r = omega_r_des,))

head_rad = 0
# head_px = 0
s_des = 0
omega_des = 0
omega_l_des = 0
omega_r_des = 0

# initialize command values
command_l = 0
command_r = 0
# initialize distance value to follow_dist_mm
# this will trick the dist_error_mm to initialize to 0
dist_mm = follow_dist_mm

try:
    start = time.time()
    while True:

        # get images (depth and color) from camera
        image_pair = vision.get_aligned_frame(pipeline)
        depth_image = image_pair[1]
        color_image = image_pair[0]

        # set camera specs
        wp = 640
        theta_fov_depth = math.radians(87)

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

            ####################################################
            #            outer loop controller                 #
            ####################################################
            # apply proportional controller to desired speed
            dist_error_mm = dist_mm - follow_dist_mm
            s_des = dist_error_mm*kp_speed

            # apply proportional controller to desired omega
            omega_des = head_rad*kp_heading

            # apply max cutoff to desired speed and omega
            s_des = min(s_des, s_max_mps)
            omega_des = min(omega_des, omega_max)

            # compute desired motor speeds
            omega_l_des = (s_des - omega_des*wheel_base_m/2) / tire_radius_m
            omega_r_des = (s_des + omega_des*wheel_base_m/2) / tire_radius_m

            # apply max cutoff to motor speed (keep vehicle from running away)
            omega_l_des = min(omega_l_des, omega_motor_max)
            omega_r_des = min(omega_r_des, omega_motor_max)

            ####################################################
            #            send actuator commands                #
            ####################################################
            # if commands are both numbers (NOT NaNs), then send build and send commands
            if not isnan(omega_l_des) and not isnan(omega_r_des):
                # if motors are on standby, take them off standby
                # if standby_flag == True:
                #     messenger.send_msg(ser,'<S,0>')
                #     standby_flag = False

                # make command strings
                command_l = '<L, ' + '%.3f'%omega_l_des + '>'
                command_r = '<R, ' + '%.3f'%omega_r_des + '>'
                # send commands
                messenger.send_msg(ser,command_l)
                messenger.send_msg(ser,command_r)

        else: # if no markers are detected
            # make command strings
            command_l = '<L,0>'
            command_r = '<R,0>'
            # send commands
            messenger.send_msg(ser,command_l)
            messenger.send_msg(ser,command_r)

            # if no markers detected, then trick controller into stopping
            s_des = 0
            omega_des = 0
            omega_l_des = 0
            omega_r_des = 0

            dist_mm = 0
            head_rad = 0

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
        print("{t:.5f}, {d:.2f}, {h:.4f},   {s:5.2f}, {o_d:5.2f},   {o_l:5.2f}, {o_r:5.2f}".format(
            t = elapsed,
            #d = dist_mm,
            d = dist_mm-follow_dist_mm,
            h = head_rad*180/3.1415,
            s = s_des,
            o_d = omega_des,
            o_l = omega_l_des,
            o_r = omega_r_des,))

finally:
    # make command strings
    command_l = '<L,0>'
    command_r = '<R,0>'
    # send commands
    messenger.send_msg(ser,command_l)
    messenger.send_msg(ser,command_r)
    messenger.send_msg(ser,'<S,1>')

    print("HIVE stopped.")
    # Stop streaming
    pipeline.stop()
    print("Stream stopped.")
