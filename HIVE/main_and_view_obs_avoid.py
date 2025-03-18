import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import imutils
import time
import math
from math import isnan
import sys
from scipy.signal import find_peaks
from time import sleep

# add directories and import custom "libraries"
sys.path.append('D:/all my files/documents/uvm/5_masters/hive/github_directory/HIVE/vision/')
import vision_continuous as vision

sys.path.append('D:/all my files/documents/uvm/5_masters/hive/github_directory/HIVE/motion/python/send_serial_motor_control/')
import send_actuation_msg as messenger
#
sys.path.append('D:/all my files/documents/uvm/5_masters/hive/github_directory/HIVE/mapping/')
from mapping import px2rad
from mapping import find_obstacles, analyze_obstacle
from mapping import filter_obstacles
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

# m/s   ft/s  mph  rad/s
# ----------------------
# 0.77  2.52  1.72  35 (near max)
# 0.66  2.16  1.48  30
# 0.55  1.80  1.23  25
# 0.33  1.08  0.74  20
# ----------------------
# max speed
s_max_mps = 0.66        # [m/s] max speed of vehicle (s/radius = omega)
omega_max = 4           # [rad/s] max omega of vehicle

omega_motor_max = 35    # [rad/s] max omega of motors
                        # 35 rad/s corresponds to nearly full actuation effort

# proportional controllers
# kp_speed = 0.0015 # 0.002 good in isolation
kp_speed = 0.0020

#kp_heading = .015 # 0.01, 0.015 good in isolation [with pixel heading]
kp_heading = 6 # 10 good in isolation (with radian heading)
#kp_heading = 10

#kp_speed = 0
#kp_heading = 0

# feedback initialize
dist_mm =  0
head_px =  0
head_rad = 0
markers = []

follow_dist_mm = 500 # nose: 250 // center: 300,


# set camera specs
wp = 640
theta_fov_depth = math.radians(87)
cam_loc = [-25,105] # sideways,forward displacement of camera (right +,fwd +)


#############
kp_speed = 0.000
kp_heading = 0
s_max_mps=0.35
# follow_dist_mm = 500
theta_fov_depth = math.radians(81)
#############

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
print('camera offset (horiz,fwd) [mm]: (' + str(cam_loc[0]) + ', ' + str(cam_loc[1]) + ')')
print('')
print('follow dist [mm]:  ' + str(follow_dist_mm))
print('')
print('kp_heading:   ' + str(kp_heading))
print('kp_speed:     ' + str(kp_speed))
print('')
print('***************************')

# print('Heading gain test. Small step (10 deg / .17 rad)')
# print('24 NOV 25')
# print('***************************')

#############################################################################################


cv2.namedWindow('HIVE Video Feeds: Raw RGB, Aligned RGB, Depth', cv2.WINDOW_AUTOSIZE)

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
        theta_fov_depth = math.radians(81)

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

            ## TEMP ADJUSTMENT TEMPORARY REMOVE FLAG
            #head_rad = head_rad + math.radians(15)

            ####################################################
            #            outer loop controller                 #
            ####################################################
            # apply proportional controller to desired speed
            dist_error_mm = dist_mm - follow_dist_mm
            s_des = dist_error_mm*kp_speed

            # apply proportional controller to desired omega
            omega_des = head_rad*kp_heading

            ####################################################
            #      interupt loop here for obs avoidance        #
            ####################################################
            # now that s and omega have been computed to get to
            # marker, need to look for obstacles. if found, need
            # to modify s and omega appropriately.
            # once modified, can continue to:
            #   0) s, omega governor check
            #   1) s, omega -> omega_l, omega_r
            #   2) omega l/r governor check
            #   3) send final omega l/r to LCMA

            num_slices = 90
            # print("looking for obs")
            obstacle_data = find_obstacles(
                depth_image,
                num_slices=num_slices,
                theta_fov_depth_hv=[math.radians(87),math.radians(58)],
                search_band=[math.radians(-30),math.radians(30)],
                visualize = False,
            )
            # obstacle data is form:
            #     0          1             2                 3           4
            # face_list, theta_list, pitch_pair_list, dist_pair_list, yaw_list
            # pitch and dist pairs are [top_edge, bottom_edge]
            # yaw gives side-to-side location

            try:
                # print("filtering obs...")
                filtered_obstacles = filter_obstacles(
                    obstacle_data,
                    thresh_face_angle=math.radians(135),
                    thresh_min_face_length=40,
                    thresh_max_face_length=150,
                    # only keep things within 80% of the distance of
                    # the currnelty seen marker
                    # thresh_distance=dist_error_mm*0.8,
                    thresh_distance=500,
                    visualize=False
                    )
                print("...obstacles filtered:")
                obstacle_yaw_bounds = [filtered_obstacles[4][0], filtered_obstacles[4][-1]]
                obstacle_pitch_bounds = filtered_obstacles[2][0]
                obstacle_center = np.average(obstacle_yaw_bounds)
                obstacle_min_dist = np.min(filtered_obstacles[3])

                # omega_des = omega_des*obstacle_center*k_avoid
                # print("omega_des old/new: {old:6.5f}, {new:6.5f}".format(old=o_old,new=omega_des))

                # "radius of safety" to put around obstacle edge
                # at minimum should be the half width of tank
                r_s = wheel_base_m*1000*0.5
                r_s = wheel_base_m*1000*0.35
                # r_s = 5

                # angle between edge of object and edge of safety bubble
                beta_s = math.asin(r_s/obstacle_min_dist)
                # print("beta[deg]: {b:.2f}".format(b=math.degrees(beta_s)))

                # dist to corner of safety bubble
                d_s = math.sqrt(r_s**2 + obstacle_min_dist**2)

                #####################
                # display obstacle stuff
                #####################
                print("about to mark")
                xpxL = int(320 + 640*math.tan(obstacle_yaw_bounds[0])/(2*math.tan(0.5*theta_fov_depth)))
                xpxR = int(320 + 640*math.tan(obstacle_yaw_bounds[1])/(2*math.tan(0.5*theta_fov_depth)))
                ypxT = int(240 - 480*math.tan(obstacle_pitch_bounds[0])/(2*math.tan(0.5*math.radians(58))))
                ypxB = int(240 - 480*math.tan(obstacle_pitch_bounds[1])/(2*math.tan(0.5*math.radians(58))))
                # cv2.circle(color_image, (xpxL, 240), 4, (0,0,255), -1)
                # cv2.circle(color_image, (xpxR, 240), 4, (0,0,255), -1)
                cv2.line(color_image, (xpxL,ypxT), (xpxR,ypxT), (0,0,255), 2)
                cv2.line(color_image, (xpxR,ypxT), (xpxR,ypxB), (0,0,255), 2)
                cv2.line(color_image, (xpxR,ypxB), (xpxL,ypxB), (0,0,255), 2)
                cv2.line(color_image, (xpxL,ypxB), (xpxL,ypxT), (0,0,255), 2)

                print("obs marked")

                if obstacle_center > 0:
                    # ^ obstacle is right of center, so turn left
                    turn_dir = "LEFT      "

                    # take left side bound and subtract to shift "more left"
                    theta_s = obstacle_yaw_bounds[0]-beta_s

                elif obstacle_center < 0:
                    # ^ obstacle is left of center, so turn right
                    turn_dir = "     RIGHT"

                    # take right side bound and add to shift "more right"
                    theta_s = obstacle_yaw_bounds[1]+beta_s
                else:
                    turn_dir = "    x     "

                # relative coordinates of point of safety
                x_s = d_s*math.sin(theta_s)
                y_s = d_s*math.cos(theta_s)
                print(x_s, y_s)

                # if abs(x_s) < wheel_base_m*1000:
                if abs(x_s) < wheel_base_m*1000/2:
                    print("might collide! adjusting setpoints")
                    # ^ if x coord of safety is within the wake of our forward travel
                    # then we need to adjust our heading
                    # (if not we can ignore obstacle)

                    # apply proportional controller to desired omega
                    # using new heading that avoids obstacle
                    omega_des = theta_s*kp_heading

                    # stop forward movement while adjusting heading
                    s_des = 0

                    # print("Obstacle center [deg]: {c:6.2f}, TURN {dir:s} to avoid".format(c=math.degrees(obstacle_center), dir=turn_dir))
                else:
                    print("not worried about collision")


                # print("obs d[mm], LCR[deg]: {d:.0f}mm,{l:9.2f}{c:9.2f}{r:9.2f}".format(
                #     d=obstacle_min_dist,
                #     l=math.degrees(obstacle_yaw_bounds[0]),
                #     c=math.degrees(obstacle_center),
                #     r=math.degrees(obstacle_yaw_bounds[1])
                # ))

            except:
                print("no obs found...")
                pass

            ####################################################
            #        apply governing checks on speeds,         #
            #            compute actuator commands             #
            ####################################################

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

        #'''
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
        images = np.hstack((color_unaligned, color_image, depth_colormap))

        # Show images
        cv2.imshow('HIVE Video Feeds: Raw RGB, Aligned RGB, Depth', images)

        k = cv2.waitKey(1) & 0xFF # escape key to stop
        if k == 27:
            break
        #'''

        end = time.time()
        elapsed = end-start
        #######################################################################################
        # print outputs #
        #######################################################################################
        '''
        print("{t:.5f}, {d:.2f}, {h:.4f},   {s:5.4f}, {o_d:5.4f},   {o_l:5.4f}, {o_r:5.4f}".format(
            t = elapsed,
            #d = dist_mm,
            d = dist_mm-follow_dist_mm,
            h = head_rad,#*180/3.1415,
            s = s_des,
            o_d = omega_des,
            o_l = omega_l_des,
            o_r = omega_r_des,))
        '''
        # input()

finally:
    # make command strings
    command_l = '<L,0>'
    command_r = '<R,0>'
    # send commands
    messenger.send_msg(ser,command_l)
    messenger.send_msg(ser,command_r)
    messenger.send_msg(ser,'<S,1>')
    print("Motors zeroed and on standby.")

    print("HIVE stopped.")
    # Stop streaming
    pipeline.stop()
    print("Stream stopped.")
