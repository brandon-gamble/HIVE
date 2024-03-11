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
s_max_mps = 0.5         # [m/s]
omega_max = 5           # [rad/s]

# proportional controllers
kp_speed = 0.001
kp_heading = 0.01 # 0.01 worked well with pixel as heading
#kp_speed = 0
#kp_heading = 0

# feedback initialize
dist_mm =  0
head_px =  0
head_rad = 0

# control timer
# t_controller =

follow_dist_mm = 250

# set camera specs
wp = 640
theta_fov_depth = math.radians(87)

####################################################
#               start serial comm                  #
####################################################
ser = messenger.initialize_com(38400);
print('waiting for connection....')
time.sleep(2)
print('***************************')
print('Vehicle and Controller Specifications')
print('***************************')
print('')
print('wheel base [m]:    ' + str(wheel_base_m))
print('tire radius [m]:   ' + str(tire_radius_m))
print('')
print('max speed [m/s]:   ' + str(s_max_mps))
print('omega max [rad/s]: ' + str(omega_max))
print('')
print('follow dist [mm]:  ' + str(follow_dist_mm))
print('')
print('kp_heading:   ' + str(kp_heading))
print('kp_speed:     ' + str(kp_speed))
print('')
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

# print("Dist [mm], Head [px], s_des [m/s], omega_l_des [rad/s], omega_r_des [rad/s], cmd_l [rad/s], cmd_r [rad/s]")
        # d = dist_mm,
        # h = head_px,
        # s = s_des,
        # o_d = omega_des,
        # o_l = omega_l_des,
        # o_r = omega_r_des,
        # cmd_l = command_l,
        # cmd_r = command_r))

print("Dist [mm], Head [rad], s_des [m/s], omega_l_des [rad/s], omega_r_des [rad/s]")
        # d = dist_mm,
        # h = head_px,
        # s = s_des,
        # o_d = omega_des,
        # o_l = omega_l_des,
        # o_r = omega_r_des,))

# initialize command values
command_l = 0
command_r = 0
# initialize distance value to follow_dist_mm
# this will trick the dist_error_mm to initialize to 0
dist_mm = follow_dist_mm

while True:
    # get images (depth and color) from camera
    image_pair = vision.get_curr_frame(pipeline)
    depth_image = image_pair[1]
    color_image = image_pair[0]

    # set camera specs
    wp = 640
    theta_fov_depth = math.radians(87)

    # detect markers in images
    markers = vision.detect_aruco(image_pair, (10, 2), visualize=False)
        # id = markers[0][0],   id,       [-]
        # x = markers[0][1][0], y,        [px]
        # y = markers[0][1][1], x,        [px]
        # d = markers[0][2],    distance, [mm]
        # h = markers[0][3]))   heading,  [px]

    if markers: # check to see if markers contains any elements (not empty)
        # convert markers from list to array
        markers = np.asarray(markers)

        # if multiple markers detected (e.g. marker on back of leader and side of leader)
        # then take the average position of them
        markers_avg = np.mean(markers, axis=0)

        # extract distance and heading detected in image
        # flip sign of heading so that targ to right gives (-),
        #   and targ to left give (+)
        dist_mm = markers_avg[3]    # distance to aruco [mm]
        head_px = -1*markers_avg[4] # heading to aruco  [px]

        # need to do some tomfoolery to convert from pixel heading to radian heading:
        # note that pixel heading has shifted the 0 of y axis to the center of the image
        # need to add back the half width of image to return to 0 at edge of image instead of center
        # px2rad also takes an array, so need to wrap heading value in array
        # finally, px2rad spits out array, so need to unwrap to just a float by indexing first value [0]
        head_rad = px2rad(np.array([head_px+wp/2]), wp, theta_fov_depth)[0]

    # else:
        # print('no markers detected')

    ####################################################
    #            outer loop controller                 #
    ####################################################
    # apply proportional controller to desired speed
    dist_error_mm = dist_mm - follow_dist_mm
    s_des = dist_error_mm*kp_speed

    # apply proportional controller to desired omega
    omega_des = head_px*kp_heading
    # omega_des = head_rad*kp_heading

    # apply max cutoff to desired speed and omega
    s_des = min(s_des, s_max_mps)
    omega_des = min(omega_des, omega_max)

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
        messenger.send_msg(ser,command_l)
        messenger.send_msg(ser,command_r)

    ####################################################
    #                 print outputs                    #
    ####################################################

    # DEBUG FORMAT
    #################################
    # print("{d:5.2f} {h:3} | {s:5.2f} {o_d:5.2f} | {o_l:5.2f} {o_r:5.2f} | {cmd_l} {cmd_r}".format(
    #     d = dist_mm,
    #     h = head_px,
    #     s = s_des,
    #     o_d = omega_des,
    #     o_l = omega_l_des,
    #     o_r = omega_r_des,
    #     cmd_l = command_l,
    #     cmd_r = command_r))

    # CSV FORMAT
    ################################
    print("{d:.2f}, {h:.4f},   {s:5.2f}, {o_d:5.2f},   {o_l:5.2f}, {o_r:5.2f}".format(
        d = dist_mm,
        # h = head_rad,
        h = head_px,
        s = s_des,
        o_d = omega_des,
        o_l = omega_l_des,
        o_r = omega_r_des,))

# Stop streaming
pipeline.stop()
print("Stream stopped")
