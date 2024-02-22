import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import imutils
import time
from math import isnan
import sys

# add directories and import custom "libraries"
sys.path.append('D:/all my files/documents/uvm/5_masters/hive/github_directory/HIVE/vision/')
import vision_continuous as vision

sys.path.append('D:/all my files/documents/uvm/5_masters/hive/github_directory/HIVE/motion/python/send_serial_motor_control/')
import send_actuation_msg as messenger
#
# sys.path.append('D:/all my files/documents/uvm/5_masters/hive/github_directory/HIVE//')
# import  as
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
s_max_mps = 0.5         # [m/s]

# proportional controllers
kp_speed = 0.001
kp_heading = 0.01

# feedback initialize
dist_mm = 0
head_px = 0

# control timer
# t_controller =

follow_dist_mm = 300

####################################################
#               start serial comm                  #
####################################################
ser = messenger.initialize_com(38400);
print('waiting for connection....')
time.sleep(2)
print('***************************')
print('COMMANDS:')
print('L, left')
print('R, right')
print('S, standby')

print('FORMAT:')
print('<L, 100>')
print('***************************')

####################################################
#               start pipeline                     #
####################################################
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# print("ID | Loc [px] | Dist [mm] | Heading [px]")
# print("----------------------------------------")
#
#     d = dist_mm,
#     h = head_px,
#     s = s_des,
#     o_d = omega_des,
#     o_l = omega_l_des,
#     o_r = omega_r_des))
while True:

    # get images (depth and color) from camera
    image_pair = vision.get_curr_frame(pipeline)

    # detect markers in images
    markers = vision.detect_aruco(image_pair, (10, 2), visualize=False)
        # id = markers[0][0],   id,       [-]
        # x = markers[0][1][0], y,        [px]
        # y = markers[0][1][1], x,        [px]
        # d = markers[0][2],    distance, [mm]
        # h = markers[0][3]))   heading,  [px]

    if markers: # check to see if markers contains any elements (not empty)
        # extract distance and heading detected in image
        # flip sign of heading so that targ to right gives (-),
        #   and targ to left give (+)
        dist_mm = markers[0][2]    # distance to aruco [mm]
        head_px = -1*markers[0][3] # heading to aruco  [px]

        # print(dist_mm)
        # print(head_px)
    # else:
        # print('no markers detected')

    ####################################################
    #            outer loop controller                 #
    ####################################################
    # apply proportional controller to desired speed
    dist_error_mm = dist_mm - follow_dist_mm
    s_des = dist_error_mm*kp_speed
    # apply max cutoff to desired speed
    s_des = min(s_des, s_max_mps)

    # apply proportional controller to desired omega
    omega_des = head_px*kp_heading

    # comptue desired motor speeds
    omega_l_des = (s_des - omega_des*wheel_base_m/2) / tire_radius_m
    omega_r_des = (s_des + omega_des*wheel_base_m/2) / tire_radius_m

    ####################################################
    #            send actuator commands                #
    ####################################################
    # if commands are both numbers (NOT NaNs), then send build and send commands
    if not isnan(omega_l_des) and not isnan(omega_r_des):
        # make command strings
        command_l = '<L, ' + '%.3f'%omega_l_des + '>'
        command_r = '<R, ' + '%.3f'%omega_r_des + '>'
        # send commands
        messenger.send_msg(ser,'command_r')
        messenger.send_msg(ser,'command_l')

    ####################################################
    #                 print outputs                    #
    ####################################################
    print("{d:5.2f} {h:3} | {s:5.2f} {o_d:5.2f} | {o_l:5.2f} {o_r:5.2f} | {cmd_l} {cmd_r}".format(
        d = dist_mm,
        h = head_px,
        s = s_des,
        o_d = omega_des,
        o_l = omega_l_des,
        o_r = omega_r_des,
        cmd_l = command_l,
        cmd_r = command_r))

# Stop streaming
pipeline.stop()
print("Stream stopped")
