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

def plot_box(image, xLR, yTB, color, weight):
    '''
    xLR = [left, right] pixel bounds
    yTB = [top, bottom] pixel bounds

    color is (b,g,r)
    std weight is 2
    '''
    xpxL, xpxR = xLR
    ypxT, ypxB = yTB
    cv2.line(image, (xpxL,ypxT), (xpxR,ypxT), color, weight)
    cv2.line(image, (xpxR,ypxT), (xpxR,ypxB), color, weight)
    cv2.line(image, (xpxR,ypxB), (xpxL,ypxB), color, weight)
    cv2.line(image, (xpxL,ypxB), (xpxL,ypxT), color, weight)
    return

def rad2px(rad, px_dim, theta_fov):
    return int(px_dim/2 - px_dim*math.tan(rad)/(2*math.tan(0.5*theta_fov)))

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
kp_speed = 0.00   # 0.002
kp_heading = 4     # 6
s_max_mps=0.2
follow_dist_mm = 250
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
            print("marker heading: {hr:.2f}".format(hr=head_rad))

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
                print("filtering obs...")
                filtered_obstacles = filter_obstacles(
                    obstacle_data,
                    thresh_face_angle=math.radians(135),
                    thresh_min_face_length=40,
                    thresh_max_face_length=150,
                    # only keep things within x% of the distance of
                    # the currnelty seen marker
                    thresh_distance=(dist_mm+cam_loc[1])*.6,
                    visualize=False
                    )
                # print("...filtered")

                # to fully account for camera location, need adjust everything
                # but for now just adjust the distance by adding the y coord of camera
                # to the distance
                obstacle_yaw_bounds = [filtered_obstacles[4][0], filtered_obstacles[4][-1]]
                obstacle_pitch_bounds = filtered_obstacles[2][0]
                obstacle_center = np.average(obstacle_yaw_bounds)
                obstacle_min_dist = np.min(filtered_obstacles[3])+cam_loc[1]

                # print("bounds computed")
                print("marker: {m:6.2f} obs: {o:6.2f}".format(m=dist_mm,o=obstacle_min_dist))

                # "radius of safety" to put around obstacle edge
                # at minimum should be the half width of tank
                r_s = wheel_base_m*1000*1.2

                # angle between edge of object and edge of safety bubble
                beta_s = math.asin(r_s/obstacle_min_dist)
                # print("beta[deg]: {b:.2f}".format(b=math.degrees(beta_s)))
                # print("beta  found")

                # expand obstacle to have margin for tank width
                yawL_safety = obstacle_yaw_bounds[0]+beta_s
                yawR_safety = obstacle_yaw_bounds[1]-beta_s

                # dist to corner of safety bubble [mm]
                # d_s = math.sqrt(r_s**2 + obstacle_min_dist**2)
                # print("ds found")

                # convert from radian bounds of obstacle to pixel bounds
                xpxL_obs = rad2px(obstacle_yaw_bounds[0], 640, theta_fov_depth)
                xpxR_obs = rad2px(obstacle_yaw_bounds[1], 640, theta_fov_depth)
                ypxT_obs = rad2px(obstacle_pitch_bounds[0], 480, math.radians(58))
                ypxB_obs = rad2px(obstacle_pitch_bounds[1], 480, math.radians(58))

                #############################
                # decide which way to turn: #
                #############################
                # dhL = head_rad - obstacle_yaw_bounds[0]
                # dhR = head_rad - obstacle_yaw_bounds[1]
                dhL = head_rad - yawL_safety
                dhR = head_rad - yawR_safety
                # if dh is positive, then marker is LEFT of edge
                # if dh is negative, then marker is RIGHT of edge
                # -----------------------------------------------
                # if both L/R are positive, then marker is LEFT of obs and WILL NOT COLLIDE
                # if .............negative,................RIGHT...........................
                # if L(-) and R(+)...............obstacle is in collision path and need to avoid
                # -----------------------------------------------
                # i.e. if both neg or both pos, can just follow heading of marker
                # but if have different signs then need to avoid obstacle

                ##########################
                # select desired heading #
                # compute pixel bounds
                ##########################
                if dhL*dhR < 0:
                    # if product is negative, then they have opposite signs
                    # and need to avoid obstacle
                    obstacle_color = (0,0,255) # make obstacle red
                    if abs(dhL) > dhR:
                        # then want to turn RIGHT
                        head_des = yawR_safety

                        # for visual:
                        # left:  obstacle edge
                        # right: expanded edge
                        xpxL_safety = rad2px(obstacle_yaw_bounds[0], 640, theta_fov_depth)
                        xpxR_safety = rad2px(yawR_safety, 640, theta_fov_depth)
                    else:
                        # want to turn left
                        head_des = yawL_safety

                        # for visual:
                        # left:  expanded edge
                        # right: obstacle edge
                        xpxL_safety = rad2px(yawL_safety, 640, theta_fov_depth)
                        xpxR_safety = rad2px(obstacle_yaw_bounds[1], 640, theta_fov_depth)
                else:
                    # if product is positive, then obstacle is not in collision
                    # path and we just take heading to the marker (head_rad)
                    head_des = head_rad

                    obstacle_color = (0,255,0) # make obstacle green

                    # for visual:
                    # NO EXPANSION
                    xpxL_safety = rad2px(obstacle_yaw_bounds[0], 640, theta_fov_depth)
                    xpxR_safety = rad2px(obstacle_yaw_bounds[1], 640, theta_fov_depth)


                # plot box around safety zone (should only extend on one side)
                plot_box(color_image, [xpxL_safety,xpxR_safety], [ypxT_obs,ypxB_obs], (51,153,255), 2)
                # plot box around obstacle
                plot_box(color_image, [xpxL_obs,xpxR_obs], [ypxT_obs,ypxB_obs], obstacle_color, 2)

                # recompute differentials with expanded safety box
                dhL_safety = head_rad + yawL_safety
                dhR_safety = head_rad + yawR_safety
                # if dh is positive, then marker is LEFT of edge
                # if dh is negative, then marker is RIGHT of edge

                omega_des = head_des*kp_heading

                print("head_mark: {hm:6.2f}, head_safe: {hs:6.2f}".format(hm=math.degrees(head_rad), hs=math.degrees(theta_s)))

                print("obs d[mm], LCR[deg]: {d:.0f}mm,{l:9.2f}{c:9.2f}{r:9.2f}".format(
                    d=obstacle_min_dist,
                    l=math.degrees(obstacle_yaw_bounds[0]),
                    c=math.degrees(obstacle_center),
                    r=math.degrees(obstacle_yaw_bounds[1])
                ))

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

        # try:
        #     if abs(x_s) < safety_bubble:
        #         # obs: red, safety: orange
        #         # plot box around safety zone (should only extend on one side)
        #         plot_box(depth_colormap, [xpxL_safety,xpxR_safety], [ypxT_obs,ypxB_obs], (51,153,255), 2)
        #         # plot box around obstacle
        #         plot_box(depth_colormap, [xpxL_obs,xpxR_obs], [ypxT_obs,ypxB_obs], (0,0,255), 2)
        #     else:
        #         plot_box(depth_colormap, [xpxL_safety,xpxR_safety], [ypxT_obs,ypxB_obs], (153, 255, 153), 2)
        #         plot_box(depth_colormap, [xpxL_obs,xpxR_obs], [ypxT_obs,ypxB_obs], (0,255,0), 2)
        # except:
        #     pass

        # mark center of color image
        cv2.circle(color_image, (320,240), 2, (252,3,248), -1)

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
