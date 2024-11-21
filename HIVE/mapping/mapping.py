import pyrealsense2.pyrealsense2 as rs
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import cm
import cv2
import argparse
import imutils
import sys
from scipy.signal import find_peaks
from time import sleep

######################################
'''
need to flip origin of image
    should probably put get_curr_frame in its own camera doc to import
    that way changes work across docs (no unique version of get frame)
    should probably flip origin in get_curr_frame, but this may break the indexing in the aruco tracking

    ** zero pixels result from "shadow" so instead of averaging to defeat zeros,
    ** need to IGNORE zeros.
'''
######################################
def get_curr_frame(pipeline):
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    # depth: <class 'pyrealsense2.pyrealsense2.depth_frame'>
    # color: <class 'pyrealsense2.pyrealsense2.video_frame'>

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    # depth: <class 'numpy.ndarray'>
    # color: <class 'numpy.ndarray'>

    frame_pair = (color_frame, depth_frame)
    image_pair = (color_image, depth_image)

    return image_pair

def get_aligned_frame(pipeline):
    align = rs.align(rs.stream.depth)

    frames = pipeline.wait_for_frames()
    frames = align.process(frames)

    aligned_color_frame = frames.get_color_frame()

    color_image = np.asanyarray(aligned_color_frame.get_data())
    depth_image = np.asanyarray(frames.get_depth_frame().get_data())

    pair = (color_image, depth_image)

    return pair

def get_z_slice_avg(depth_image, slice_height, slice_size):
    # depth_image:  array of depth data
    # slice_height: z location in image to center slice
    # slice_size:   how thick slice to be

    # will fail if slice goes past edge of image.

    # since indexing starts from top of array and moves down,
    # we need to index row from bottom, hence the (-) sign
    dz = int(slice_size/2) # truncate decimal
    z_start = -slice_height - dz
    z_end = z_start + slice_size
    z_slice = depth_image[z_start:z_end:,:]

    # now need to average down to a single row
    z_slice = np.mean(z_slice, axis=0)

    return z_slice

def get_z_slice(depth_image, slice_idx_px):
    # depth_image:  array of depth data
    # slice_idx_px: z location in image to center slice
    #               (0 is top of image)

    z_slice = depth_image[slice_idx_px,:]

    return z_slice

def plot_image_3d(image):
    # "image" is array, not frame

    image = np.flip(image, 0)

    height = image.shape[0]
    width = image.shape[1]

    x = np.arange(0,width)
    y = np.arange(0,height)
    X, Y = np.meshgrid(x, y)

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.set_xlabel('$x$')
    ax.set_ylabel('$y$')

    surf = ax.plot_surface(X, Y, image, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)


    plt.show()

    return

def plot_image_2d(image, axes="px"):
    # "image" is array, not frame

    # good cmap options: inferno, hsv
    if axes == "px":
        plt.imshow(image, cmap='inferno', vmin=0, vmax=1500)
        plt.colorbar()
        plt.title('Realsense FOV, Depth [mm]')
        plt.xlabel('x [px]')
        plt.ylabel('z [px]')
    elif axes == "rad":
        plt.imshow(image, cmap='inferno', vmin=0, vmax=1500,
            extent=[-math.radians(87/2),math.radians(87/2),-math.radians(58/2),math.radians(58/2)])
        plt.colorbar()
        plt.title('Realsense FOV, Depth [mm]')
        plt.xlabel('yaw [rad]')
        plt.ylabel('pitch [rad]')
    elif axes == "deg":
        plt.imshow(image, cmap='inferno', vmin=0, vmax=1500,
            extent=[-87/2,87/2,-58/2,58/2])
        plt.colorbar()
        plt.title('Realsense FOV, Depth [mm]')
        plt.xlabel('yaw [deg]')
        plt.ylabel('pitch [deg]')
    plt.show()
    return

def px2rad(px, wp, theta_fov):
    '''
    --------------------------------------------------------
    input
    --------------------------------------------------------
    px:        np.array  [px]   pixel coordinate
    wp:        int       [px]   width of image
    theta_fov: int       [rad]  angle of field of view

    --------------------------------------------------------
    output
    --------------------------------------------------------
    theta:     np.array  [rad]  heading of pixel

    '''

    # "focal distance" / apparent distance of pixels
    dp = wp/(2*np.tan(theta_fov/2))

    theta = np.arctan((px-wp/2)/dp)

    return theta

def analyze_obstacle(theta1, theta2, d1, d2):
    '''
    input:
    theta1  angle to bottom edge of obstacle
    theta2    ''     top     ''   ''  ''
    d1      dist to  bottom  ''   ''  ''
    d2       ''  ''  top     ''   ''  ''

    i.e. #1 is bottom edge, #2 is top edge.
    obstacle defined by distance and angle to edges scanning from bottom up

    output:
    f       length of face of obstacle (dist b/t points defined by [theta1,d1] and [theta2,d2])
    theta_obs pitch of obstacle, where 90 deg is vertical, <90 leans toward, >90 leans away

    **** angles measured in radians ****
    '''

    # visual angle of obstacle
    alpha = theta2 - theta1
    # if theta2 < 0:
    #     alpha = abs(theta1) - abs(theta2)
    # else:
    #     alpha = abs(theta1) + abs(theta2)
    # length of face of obstacle
    f = math.sqrt(d1**2 + d2**2 - 2*d1*d2*math.cos(alpha))
    # lower internal angle of visual triangle
    # theta3 = math.asin(d2/f*math.sin(alpha)) # law of sines
    # law of sines sometimes returns the supplement of theta3, which is bad!
    theta3 = math.acos((d1**2+f**2-d2**2)/(2*d1*f)) # law of cosines
    # law of cosines gives proper theta3 for all cases....(right?)
    # pitch of obstacle:
        # 90 deg == 1.5708 rad
        # =90  is vertical
        # <90 is leaning toward
        # >90 is leaning away
    theta_obs = theta3-theta1

    return f, theta_obs

def tester(t1,d1,t2,d2):
    # this is just a helper func for case 13
    f,t = analyze_obstacle(math.radians(t1),math.radians(t2),d1,d2)
    t=math.degrees(t)
    return f,t

def num_to_range(num, inMin, inMax, outMin, outMax):
  return outMin + (float(num - inMin) / float(inMax - inMin) * (outMax - outMin))

# def display_obstacles(depth_image, obstacles, theta_fov_depth_hv=[math.radians(87),math.radians(58)]):
#     # image size
#     wp = np.shape(depth_image)[1] # usually 640
#     hp = np.shape(depth_image)[0] # usually 480
#
#     # camera specs
#     theta_fov_depth_horiz = theta_fov_depth_hv[0]
#     theta_fov_depth_vert = theta_fov_depth_hv[1]
#
#     # convert vertical pixels to headings
#     y_array = np.arange(hp)
#     y_array = px2rad(y_array, hp, theta_fov_depth_vert)
#     # flip orientation because y px index starts at 0 at top and increases
#     # as you move down. want angle to be 0 at level and increase as you
#     # tilt up
#
#     # this is an array of radian values across vertical axis of image (pitch)
#     y_array = np.flip(y_array)
#
#     # compute slice bounds (radians)
#     minthetafov = -theta_fov_depth_horiz/2
#     maxthetafov = +theta_fov_depth_horiz/2
#
#     plt.imshow(depth_image, cmap='inferno', vmin=0, vmax=1500,
#         extent=[
#             -theta_fov_depth_horiz/2,
#             theta_fov_depth_horiz/2,
#             -theta_fov_depth_vert/2,
#             theta_fov_depth_vert/2
#         ]
#     )
#     plt.colorbar()
#     plt.title('Realsense FOV, Depth [mm]. Obstacle Detection')
#     plt.xlabel('yaw [rad]')
#     plt.ylabel('pitch [rad]')
#
#
#     for k,obs in enumerate(obstacles):
#         loc = obstacles[4][k]
#         pitch_pair = obstacles[2][k]
#         face = obstacles[0][k]
#         theta = obstacles[1][k]
#
#         # print face length estimation on depth image
#         # plt.text(px2rad(loc, wp, theta_fov_depth_horiz), pitch_pair[0], f"f={face:3.1f},th={math.degrees(theta):4.1f}", color='cyan')
#         plt.text(loc, pitch_pair[0], f"f={face:3.1f},th={math.degrees(theta):4.1f}", color='cyan')
#
#         # print("face length = " + str(f))
#         # print("angle = " + str(math.degrees(theta)))
#
#         # for peak in peaks:
#         #     # convert slice location to radian, this is the x-coord in plot we are looking at
#         #     # data[0,peak] is the radian value along the vertical slice that peak occurs at
#         #     plt.plot(px2rad(loc, wp, theta_fov_depth_horiz), data[0,peak], "cx")
#
#     plt.show()
#
#     return

def find_obstacles(
    depth_image,
    num_slices,
    theta_fov_depth_hv=[math.radians(87),math.radians(58)],
    search_band=[math.radians(-20),math.radians(20)],
    visualize = False,
    ):

    '''
    inputs:
    ----------------------------------------------
    depth_image                                 depth image from pipeline
    theta_fov_depth_hv  [width, height] [rad]   fov of camera
    search_band         [left, right]   [rad]   from left to right, where in fov you want to scan
    num_slices          int                     how many slices to take in search_band

    outputs:
    ----------------------------------------------
    face_list           [face_length,...]           list of obstacle face lengths
    theta_list          [theta_obs,...]             list of obstacle angles
    pitch_pair_list     [[theta_top, theta_bot],...]list of pairs of angles to bot and top edges of obstacles
    dist_pair_list      [[dist_top, dist_bot],...]  list of pairs of distances to bot and top edges of obstacles
    yaw_list
    '''
    # image size
    wp = np.shape(depth_image)[1] # usually 640
    hp = np.shape(depth_image)[0] # usually 480

    # camera specs
    theta_fov_depth_horiz = theta_fov_depth_hv[0]
    theta_fov_depth_vert = theta_fov_depth_hv[1]

    # convert vertical pixels to headings
    y_array = np.arange(hp)
    y_array = px2rad(y_array, hp, theta_fov_depth_vert)
    # flip orientation because y px index starts at 0 at top and increases
    # as you move down. want angle to be 0 at level and increase as you
    # tilt up

    # this is an array of radian values across vertical axis of image (pitch)
    y_array = np.flip(y_array)

    # compute slice bounds (radians)
    minthetafov = -theta_fov_depth_horiz/2
    maxthetafov = +theta_fov_depth_horiz/2

    # bounds to pixels
    slice_loc_min_px = int(num_to_range(search_band[0], minthetafov, maxthetafov, 0, wp))
    slice_loc_max_px = int(num_to_range(search_band[1], minthetafov, maxthetafov, 0, wp))
    step_size = int((slice_loc_max_px-slice_loc_min_px)/num_slices)

    # set slice locations (pixels)
    slice_locs = list(range(slice_loc_min_px, slice_loc_max_px, step_size))

    if visualize == True:
        # plot depth image
        plt.imshow(depth_image, cmap='inferno', vmin=0, vmax=1500,
            extent=[
                -theta_fov_depth_horiz/2,
                theta_fov_depth_horiz/2,
                -theta_fov_depth_vert/2,
                theta_fov_depth_vert/2
            ]
        )
        plt.colorbar()
        plt.title('Realsense FOV, Depth [mm]. Obstacle Detection')
        plt.xlabel('yaw [rad]')
        plt.ylabel('pitch [rad]')

    # initialize obstacle outputs
    face_list = []
    theta_list = []
    pitch_pair_list = []
    dist_pair_list = []
    yaw_list = []

    for loc in slice_locs:

        # print("slice loc: " + str(px2rad(loc, wp, theta_fov_depth_horiz)))
        # get vertical slice (constant x)
        slice = depth_image[:,loc]

        # put into pairs [rad,depth]
        data = np.array([y_array, slice])

        # remove zeros
        mask = data[1,:] != 0
        data = data[:,mask]

        # if data is not empty from masking zeros, continue
        if data.shape[1] > 2:
            # find slope (proper derivative)
            slope = np.gradient(data[1,:], data[0,:]) # [mm/rad]

            ##############
            # find peaks #
            ##############
            peaks, _ = find_peaks(slope, height=100, distance=20, prominence=1000)
            # peaks is index into slope - index 0 is top of image
            # slope is 1D array deriv of data
            # data is [rad, depth] pairs

            if len(peaks) >= 2:
                # we only want two peaks: the bottom and top edges of Obstacle
                # therefore if there are extraneous peaks, we slice just the "first" two,
                # first with respect to scanning from bottom to top
                # since pixel coords go from top to bottom, we take
                # the last two peaks instead of the first two
                peaks = peaks[-2:]

                bump = 10
                # peaks[0] is the top edge index
                # bump it down a bit to make sure it is on the obstacle face
                # and not on the background
                peaks[0] += bump
                # bump bottom edge up a bit to make sure it is on face
                # and not on foreground
                peaks[1] -= bump

                # mask data again
                pitch_pair = data[0,peaks] # pitch coordinates, i.e. y-coord
                dist_pair = data[1,peaks]  # distances found at given coords

                face, theta = analyze_obstacle(pitch_pair[1], pitch_pair[0], dist_pair[1], dist_pair[0])

                face_list.append(face)
                theta_list.append(theta)
                pitch_pair_list.append(pitch_pair)
                dist_pair_list.append(dist_pair)
                yaw_list.append(px2rad(loc, wp, theta_fov_depth_horiz))

                if visualize == True:
                    print("slice loc: {s:4.1f}".format(s=(px2rad(loc, wp, theta_fov_depth_horiz))))
                    # print(pitch_pair)
                    print("dist: {d1:5.2f}, {d2:5.2f}".format(d1=dist_pair[0],d2=dist_pair[1]))
                    print("face: {f:5.2f}".format(f=face))
                    print("theta_obs: {t:4.1f}".format(t=math.degrees(theta)))
                    print("----------------------")
                    # '''
                    # print face length estimation on depth image
                    plt.text(px2rad(loc, wp, theta_fov_depth_horiz), pitch_pair[0], f"f={face:3.1f},th={math.degrees(theta):4.1f}", color='cyan')

                    # print("face length = " + str(f))
                    # print("angle = " + str(math.degrees(theta)))

                    for peak in peaks:
                        # convert slice location to radian, this is the x-coord in plot we are looking at
                        # data[0,peak] is the radian value along the vertical slice that peak occurs at
                        plt.plot(px2rad(loc, wp, theta_fov_depth_horiz), data[0,peak], "cx")

    if visualize == True:
        plt.show()


    return face_list, theta_list, pitch_pair_list, dist_pair_list, yaw_list

def filter_obstacles(
    obstacle_data,
    thresh_face_angle=math.radians(135),
    thresh_min_face_length=40,
    thresh_max_face_length=200,
    thresh_distance=1000,
    visualize=False
    ):

    '''
    obstacle data is form:
    0          1             2                 3           4
    face_list, theta_list, pitch_pair_list, dist_pair_list, yaw_list

    pitch and dist pairs are [top_edge, bottom_edge]

    yaw gives side-to-side location

    '''

    # thresh_face_angle
    # thresh_distance

    # put lists into arrays to be able to mask
    face_array = np.array(obstacle_data[0])         # face lengths
    theta_array = np.array(obstacle_data[1])        # face angles
    pitch_pair_array = np.array(obstacle_data[2])   # angles to bot and top edges
    dist_pair_array = np.array(obstacle_data[3])    # dists to bot and top edges
    yaw_array = np.array(obstacle_data[4])          # yaw location of obstacle

    #######################################
    #       MASK 1a - FACE LENGTH MIN     #
    # keep only things bigger than thresh #
    #######################################
    face_mask = face_array > thresh_min_face_length

    face_array = face_array[face_mask]
    theta_array = theta_array[face_mask]
    pitch_pair_array = pitch_pair_array[face_mask]
    dist_pair_array = dist_pair_array[face_mask]
    yaw_array = yaw_array[face_mask]

    #######################################
    #       MASK 1b - FACE LENGTH MAX     #
    # keep only things bigger than thresh #
    #######################################
    face_mask = face_array < thresh_max_face_length

    face_array = face_array[face_mask]
    theta_array = theta_array[face_mask]
    pitch_pair_array = pitch_pair_array[face_mask]
    dist_pair_array = dist_pair_array[face_mask]
    yaw_array = yaw_array[face_mask]

    #######################################
    #          MASK 2- FACE ANGLE         #
    #  keep only things less than thresh  #
    #######################################
    angle_mask = theta_array < thresh_face_angle

    face_array = face_array[angle_mask]
    theta_array = theta_array[angle_mask]
    pitch_pair_array = pitch_pair_array[angle_mask]
    dist_pair_array = dist_pair_array[angle_mask]
    yaw_array = yaw_array[angle_mask]

    #######################################
    #      MASK 3 - OBSTACLE DISTANCE     #
    # keep only things closer than thresh #
    #######################################
    # take the larger distance of the top/bot edges of obstacle
    # if the further edge is still within thresh, then keep
    dist_mask = np.max(dist_pair_array,axis=1) < thresh_distance

    face_array = face_array[dist_mask]
    theta_array = theta_array[dist_mask]
    pitch_pair_array = pitch_pair_array[dist_mask]
    dist_pair_array = dist_pair_array[dist_mask]
    yaw_array = yaw_array[dist_mask]

    # face_array = face_array[]
    # theta_array = theta_array[]
    # pitch_pair_array = pitch_pair_array[]
    # dist_pair_array = dist_pair_array[]

    return face_array, theta_array, pitch_pair_array, dist_pair_array, yaw_array

def main():
    '''
    test files
    ---------------------------
    01  plot color map of depth image, then aligned color image.
        note that color image has smaller FOV, so when aligned has large black border
    02  plot:
            - 2d color map,
            - 3d color map (not really useful),
            - color image (aligned)
            - slices from map
    03  3 horizontal slices of depth
    04  3 vertical   slices of depth
    05  # vertical   slices of depth
    06  1 vertical   slices of depth AND "deriv" AND slope
    07  1 vertical   slices of depth AND "deriv" AND slope SMOOTHING comparison: kernels= [5,10,20,40,100]
    08  1 vertical   slices of depth AND "deriv" AND slope SMOOTHING comparison: kernels= [5,10,20]
    09  1 vertical   slices of depth, difference, slope. -> peak finder
    10  many vert slices, find peaks (edges in environment) and overlay on depth image
    11  takes peaks and package into pairs of edge vertices. estimate obstacle size. show on plot
    12  same as 11 but also displays face angle [deg]
    13  fixing flaw in analyze_obstacle function (giving supplementary angle of theta3 and therefore incorrect theta_obs)
    14  wrap obstacle detect stuff from cases 10-12 into a function and test
    15  look at graph of filtered face sizes and gradient of face sizes
    16  look at obstacle data (unfiltered)
    17  look at obstacle data (filtered)
    18  (static) filtered obstacles -> max/min yaw (in order to go around)
    19  (dynamic) get obstacle bounds (yaw i.e. left/right) and print which way you want to turn
    '''
    test_case = 18
    print("**************************")
    print("*      TEST CASE " + str(test_case) + "      *")
    print("**************************")

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    if test_case == 1:
        while True:
            #image_pair = get_curr_frame(pipeline)
            image_pair = get_aligned_frame(pipeline)
            depth_image = image_pair[1]
            color_image = image_pair[0]

            plot_image_2d(depth_image)
            plt.imshow(color_image)
            plt.show()

    elif test_case == 2:
        while True:
            image_pair = get_aligned_frame(pipeline)
            depth_image = image_pair[1]
            color_image = image_pair[0]

            plot_image_2d(depth_image)
            plot_image_3d(depth_image)

            plt.imshow(color_image)
            plt.show()

            # set slice locations
            a = 170
            b = 340
            c = 400

            # get slices
            z_slice_a = get_z_slice(depth_image, a)
            z_slice_b = get_z_slice(depth_image, b)
            z_slice_c = get_z_slice(depth_image, c)

            # remove zeros from slices
            z_slice_a = z_slice_a[z_slice_a != 0]
            z_slice_b = z_slice_b[z_slice_b != 0]
            z_slice_c = z_slice_c[z_slice_c != 0]

            # plot slices
            plt.plot(z_slice_a, label=str(a)+' [px]')
            plt.plot(z_slice_b, label=str(b)+' [px]')
            plt.plot(z_slice_c, label=str(c)+' [px]')

            plt.xlabel('x [px]')
            plt.ylabel('depth [mm]')
            plt.title('Realsense FOV depth slice [mm]')
            plt.legend()
            plt.show()
    elif test_case == 3:
        while True:
            # get images
            image_pair = get_aligned_frame(pipeline)
            depth_image = image_pair[1]
            color_image = image_pair[0]

            # set camera specs
            wp = 640
            theta_fov_depth = math.radians(87)

            # plot depth image
            plot_image_2d(depth_image)

            # plot color image
            plt.imshow(color_image)
            plt.show()

            # convert horizontal pixels to headings
            x_array = np.arange(wp)
            x_array = px2rad(x_array, wp, theta_fov_depth)

            # set slice locations
            # a_px = 150
            a_px = 175
            b_px = 225
            c_px = 300

            # get slices
            z_slice_a = get_z_slice(depth_image, a_px)
            z_slice_b = get_z_slice(depth_image, b_px)
            z_slice_c = get_z_slice(depth_image, c_px)

            # put into pairs [rad, depth]
            data_a = np.array([x_array,z_slice_a])
            data_b = np.array([x_array,z_slice_b])
            data_c = np.array([x_array,z_slice_c])

            # remove zeros from each set
            mask_a = data_a[1,:] != 0
            data_a = data_a[:, mask_a]

            mask_b = data_b[1,:] != 0
            data_b = data_b[:, mask_b]

            mask_c = data_c[1,:] != 0
            data_c = data_c[:, mask_c]

            # plot masked slices
            plt.plot(np.degrees(data_a[0,:]), data_a[1,:], label=str(a_px)+' [px]')
            plt.plot(np.degrees(data_b[0,:]), data_b[1,:], label=str(b_px)+' [px]')
            plt.plot(np.degrees(data_c[0,:]), data_c[1,:], label=str(c_px)+' [px]')

            plt.xlabel('yaw [deg]')
            plt.ylabel('depth [mm]')
            plt.legend()
            plt.title('Realsense FOV depth slice [mm]')
            plt.show()
    elif test_case == 4:
        while True:
            # get images
            image_pair = get_aligned_frame(pipeline)
            depth_image = image_pair[1]
            color_image = image_pair[0]

            # set camera specs
            wp = 640
            hp = 480
            theta_fov_depth_horiz = math.radians(87)
            theta_fov_depth_vert = math.radians(58)

            # plot color image
            plt.imshow(color_image)
            plt.show()

            # plot depth image
            plot_image_2d(depth_image)

            # convert vertical pixels to headings
            y_array = np.arange(hp)
            y_array = px2rad(y_array, hp, theta_fov_depth_vert)
            # flip orientation because y px index starts at 0 at top and increases
            # as you move down. want angle to be 0 at level and increase as you
            # titt up
            y_array = np.flip(y_array)

            # set slice locations
            a_px = 290
            b_px = 290
            c_px = 290

            # get vertical slices (constant x)
            x_slice_a = depth_image[:,a_px]
            x_slice_b = depth_image[:,b_px]
            x_slice_c = depth_image[:,c_px]

            # put into pairs [rad, depth]
            data_a = np.array([y_array,x_slice_a])
            data_b = np.array([y_array,x_slice_b])
            data_c = np.array([y_array,x_slice_c])

            # remove zeros from each set
            mask_a = data_a[1,:] != 0
            data_a = data_a[:, mask_a]

            mask_b = data_b[1,:] != 0
            data_b = data_b[:, mask_b]

            mask_c = data_c[1,:] != 0
            data_c = data_c[:, mask_c]

            # plot masked slices
            plt.plot(np.degrees(data_a[0,:]), data_a[1,:], '.', label=str(a_px)+' [px]')
            plt.plot(np.degrees(data_b[0,:]), data_b[1,:], '.', label=str(b_px)+' [px]')
            plt.plot(np.degrees(data_c[0,:]), data_c[1,:], '.', label=str(c_px)+' [px]')

            plt.xlabel('pitch [deg]')
            plt.ylabel('depth [mm]')
            plt.legend()
            plt.title('Realsense FOV vertical depth slice [mm]')
            plt.show()
    elif test_case == 5:
        while True:
            # get images
            image_pair = get_aligned_frame(pipeline)
            depth_image = image_pair[1]
            color_image = image_pair[0]

            # set camera specs
            wp = 640
            hp = 480
            theta_fov_depth_horiz = math.radians(87)
            theta_fov_depth_vert = math.radians(58)

            # plot color image
            plt.imshow(color_image)
            plt.show()

            # plot depth image
            plot_image_2d(depth_image)

            # convert vertical pixels to headings
            y_array = np.arange(hp)
            y_array = px2rad(y_array, hp, theta_fov_depth_vert)
            # flip orientation because y px index starts at 0 at top and increases
            # as you move down. want angle to be 0 at level and increase as you
            # titt up
            y_array = np.flip(y_array)

            # set slice locations
            slice_locs = [290, 300, 310, 320, 330, 340]

            for loc in slice_locs:
                # get vertical slice (constant x)
                slice = depth_image[:,loc]

                # put into pairs [rad,depth]
                data = np.array([y_array, slice])

                # remove zeros
                mask = data[1,:] != 0
                data = data[:,mask]

                # plot
                plt.plot(np.degrees(data[0,:]), data[1,:], '.', label=str(loc)+' [px]')

            plt.xlabel('pitch [deg]')
            plt.ylabel('depth [mm]')
            plt.legend()
            plt.title('Realsense FOV vertical depth slice [mm]')
            plt.show()
    elif (test_case == 6) or (test_case == 7) or (test_case == 8):
        while True:
            # get images
            image_pair = get_aligned_frame(pipeline)
            depth_image = image_pair[1]
            color_image = image_pair[0]

            # set camera specs
            wp = 640
            hp = 480
            theta_fov_depth_horiz = math.radians(87)
            theta_fov_depth_vert = math.radians(58)

            # plot color image
            plt.imshow(color_image)
            plt.show()

            # plot depth image
            plot_image_2d(depth_image)

            # convert vertical pixels to headings
            y_array = np.arange(hp)
            y_array = px2rad(y_array, hp, theta_fov_depth_vert)
            # flip orientation because y px index starts at 0 at top and increases
            # as you move down. want angle to be 0 at level and increase as you
            # titt up
            y_array = np.flip(y_array)

            fig, axs = plt.subplots(3,1)

            # set slice locations
            slice_locs = [290, 300, 310, 320, 330, 340]
            slice_locs = [217, 390]
            slice_locs = [217]

            for loc in slice_locs:
                # get vertical slice (constant x)
                slice = depth_image[:,loc]

                # put into pairs [rad,depth]
                data = np.array([y_array, slice])

                # remove zeros
                mask = data[1,:] != 0
                data = data[:,mask]

                # find diff between points (not quite derivative because of
                # non uniform spacing due to mask)
                nghbr_pxl_diff = np.diff(data[1,:])

                # find slope (proper derivative)
                slope = np.gradient(data[1,:], data[0,:]) # [mm/rad]
                # slope = np.degrees(slope)                 # [mm/deg]

                if test_case == 7:
                    ##############################
                    # smoothing the slope signal #
                    ##############################
                    kernel_sizes = [5,10,20,40,100]
                    for k_size in kernel_sizes:
                        kernel = np.ones(k_size) / k_size
                        slope_smooth = np.convolve(slope, kernel, mode='same')
                        axs[2].plot(np.degrees(data[0,:]), slope_smooth, label='k='+str(k_size))

                if test_case == 8:
                    ##############################
                    # smoothing the slope signal #
                    ##############################
                    kernel_sizes = [5,10,20]
                    for k_size in kernel_sizes:
                        kernel = np.ones(k_size) / k_size
                        slope_smooth = np.convolve(slope, kernel, mode='same')
                        axs[2].plot(np.degrees(data[0,:]), slope_smooth, label='k='+str(k_size))

                # plot slice
                axs[0].plot(np.degrees(data[0,:]), data[1,:], '.', label=str(loc)+' [px]')

                # plot "derive"
                axs[1].plot(np.degrees(data[0,1:]), nghbr_pxl_diff, label=str(loc)+' [px]')

                # plot slope
                axs[2].plot(np.degrees(data[0,:]), slope, label=str(loc)+' [px]')

            axs[0].set_xlabel('pitch [deg]')
            axs[1].set_xlabel('pitch [deg]')
            axs[2].set_xlabel('pitch [deg]')

            axs[0].set_ylabel('depth [mm]')
            axs[1].set_ylabel('difference between neighbors [mm]')
            axs[2].set_ylabel('slope [mm/rad]')

            # axs[0].title.set_text('Slice depth')
            # axs[1].title.set_text('Pixel to Pixel change')
            # axs[2].title.set_text('Slope')
            axs[0].title.set_text('Obstacle Analysis: Vertical Depth Slices')

            axs[0].legend()

            axs[2].legend()

            plt.show()
    elif test_case == 9:
        '''
        - results show that the raw data gives better peaks.
        - this is great because we can therefore use the raw data and avoid
            the convolution used for smoothing.
        - good peak parameters: height=100, distance=20, prominence=1000
        '''
        while True:
            # get images
            image_pair = get_aligned_frame(pipeline)
            depth_image = image_pair[1]
            color_image = image_pair[0]

            # set camera specs
            wp = 640
            hp = 480
            theta_fov_depth_horiz = math.radians(87)
            theta_fov_depth_vert = math.radians(58)

            # plot color image
            plt.imshow(color_image)
            plt.title('Realsense FOV, Depth [mm]')
            plt.xlabel('x [px]')
            plt.ylabel('z [px]')
            plt.show()

            # plot depth image
            plot_image_2d(depth_image, axes="deg")

            # convert vertical pixels to headings
            y_array = np.arange(hp)
            y_array = px2rad(y_array, hp, theta_fov_depth_vert)
            # flip orientation because y px index starts at 0 at top and increases
            # as you move down. want angle to be 0 at level and increase as you
            # titt up
            y_array = np.flip(y_array)

            fig, axs = plt.subplots(3,1)

            # set slice locations
            slice_locs = [290, 300, 310, 320, 330, 340]
            slice_locs = [217, 390]
            slice_locs = [350]

            for loc in slice_locs:
                # get vertical slice (constant x)
                slice = depth_image[:,loc]

                # put into pairs [rad,depth]
                data = np.array([y_array, slice])

                # remove zeros
                mask = data[1,:] != 0
                data = data[:,mask]

                # find diff between points (not quite derivative because of
                # non uniform spacing due to mask)
                nghbr_pxl_diff = np.diff(data[1,:])

                # find slope (proper derivative)
                slope = np.gradient(data[1,:], data[0,:]) # [mm/rad]
                # slope = np.degrees(slope)                 # [mm/deg]

                ##############################
                # smoothing the slope signal #
                ##############################
                kernel_size = 5
                kernel = np.ones(kernel_size) / kernel_size
                slope_smooth = np.convolve(slope, kernel, mode='same')

                #plot slope
                axs[1].plot(np.degrees(data[0,:]), slope, label=str(loc)+' [px], raw')
                axs[2].plot(np.degrees(data[0,:]), slope_smooth, label=str(loc)+' [px], k='+str(kernel_size))

                # plot slice
                axs[0].plot(np.degrees(data[0,:]), data[1,:], '.', label=str(loc)+' [px]')

                ##############
                # find peaks #
                ##############
                peaks, _ = find_peaks(slope, height=100, distance=20, prominence=1000)
                axs[1].plot(np.degrees(data[0,peaks]), slope[peaks], "x")

                peaks_smooth, _ = find_peaks(slope_smooth, height=100, distance=20, prominence=1000)
                axs[2].plot(np.degrees(data[0,peaks_smooth]), slope_smooth[peaks_smooth], "x")

            axs[0].set_xlabel('pitch [deg]')
            axs[1].set_xlabel('pitch [deg]')
            axs[2].set_xlabel('pitch [deg]')

            axs[0].set_ylabel('depth [mm]')
            axs[1].set_ylabel('slope [mm/rad]')
            axs[2].set_ylabel('slope [mm/rad]')

            # axs[0].title.set_text('Slice depth')
            # axs[1].title.set_text('Pixel to Pixel change')
            # axs[2].title.set_text('Slope')
            axs[0].title.set_text('Obstacle Analysis: Vertical Depth Slices')

            axs[0].legend()
            axs[1].legend()
            axs[2].legend()

            plt.show()
    elif test_case == 10:
        # get images
        image_pair = get_aligned_frame(pipeline)
        # capture a second time so auto exposure has a chance to activate
        image_pair = get_aligned_frame(pipeline)
        depth_image = image_pair[1]
        color_image = image_pair[0]

        # set camera specs
        wp = 640
        hp = 480
        theta_fov_depth_horiz = math.radians(87)
        theta_fov_depth_vert = math.radians(58)

        # plot color image
        plt.imshow(color_image)
        plt.title('Realsense FOV, Depth [mm]')
        plt.xlabel('x [px]')
        plt.ylabel('z [px]')
        plt.show()

        while True:
            # get images
            image_pair = get_aligned_frame(pipeline)
            depth_image = image_pair[1]
            color_image = image_pair[0]

            # convert vertical pixels to headings
            y_array = np.arange(hp)
            y_array = px2rad(y_array, hp, theta_fov_depth_vert)
            # flip orientation because y px index starts at 0 at top and increases
            # as you move down. want angle to be 0 at level and increase as you
            # titt up
            y_array = np.flip(y_array)

            # fig, axs = plt.subplots(2,1)

            # set slice locations (pixels)
            slice_locs = [225,250,275,300,325,350, 430,440,450,460,470,480,490,500]
            slice_locs = list(range(0,wp,20))

            # plot color image
            plt.imshow(depth_image, cmap='inferno', vmin=0, vmax=1500,
                # extent=[-87/2,87/2,-58/2,58/2])
                extent=[-math.radians(87/2),math.radians(87/2),-math.radians(58/2),math.radians(58/2)])
            plt.colorbar()
            plt.title('Realsense FOV, Depth [mm]. Obstacle Detection')
            plt.xlabel('yaw [rad]')
            plt.ylabel('pitch [rad]')
            for loc in slice_locs:
                # get vertical slice (constant x)
                slice = depth_image[:,loc]

                # put into pairs [rad,depth]
                data = np.array([y_array, slice])

                # remove zeros
                mask = data[1,:] != 0
                data = data[:,mask]

                # if data is not empty from masking zeros, continue
                if data.shape[1] > 2:
                    # find slope (proper derivative)
                    slope = np.gradient(data[1,:], data[0,:]) # [mm/rad]

                    ##############
                    # find peaks #
                    ##############
                    peaks, _ = find_peaks(slope, height=100, distance=20, prominence=1000)
                    # peaks is index into slope
                    # slope is 1D array deriv of data
                    # data is [rad, depth] pairs

                    if len(peaks) >= 2:
                        # we only want two peaks: the bottom and top edges of Obstacle
                        # therefore if there are extraneous peaks, we slice just the "first" two,
                        # first with respect to scanning from bottom to top
                        # since pixel coords go from top to bottom, we take
                        # the last two peaks instead of the first two
                        peaks = peaks[-2:]

                        for peak in peaks:
                            # ignore peaks that are taller than threshold
                            if data[0,peak] < math.radians(6):
                                # convert slice location to radian, this is the x-coord in plot we are looking at
                                # data[0,peak] is the radian value along the vertical slice that peak occurs at
                                plt.plot(px2rad(loc, wp, theta_fov_depth_horiz), data[0,peak], "cx")
            plt.show()
    elif (test_case == 11) or (test_case == 12):
        # get images
        image_pair = get_aligned_frame(pipeline)
        sleep(0.5)
        # capture a second time so auto exposure has a chance to activate
        image_pair = get_aligned_frame(pipeline)
        depth_image = image_pair[1]
        color_image = image_pair[0]

        # set camera specs
        wp = 640
        hp = 480
        theta_fov_depth_horiz = math.radians(87)
        theta_fov_depth_vert = math.radians(58)

        # plot color image
        plt.imshow(color_image)
        plt.title('Realsense FOV, Depth [mm]')
        plt.xlabel('x [px]')
        plt.ylabel('z [px]')
        plt.show()

        while True:
            print("----- NEW SCAN -----")
            # get images
            image_pair = get_aligned_frame(pipeline)
            depth_image = image_pair[1]
            color_image = image_pair[0]

            # convert vertical pixels to headings
            y_array = np.arange(hp)
            y_array = px2rad(y_array, hp, theta_fov_depth_vert)
            # flip orientation because y px index starts at 0 at top and increases
            # as you move down. want angle to be 0 at level and increase as you
            # tilt up
            y_array = np.flip(y_array)

            # set slice locations (pixels)
            slice_locs = list(range(0,wp,20))
            slice_locs = [300, 450]
            slice_locs = [350]

            # plot depth image
            plt.imshow(depth_image, cmap='inferno', vmin=0, vmax=1500,
                extent=[-math.radians(87/2),math.radians(87/2),-math.radians(58/2),math.radians(58/2)])
            plt.colorbar()
            plt.title('Realsense FOV, Depth [mm]. Obstacle Detection')
            plt.xlabel('yaw [rad]')
            plt.ylabel('pitch [rad]')

            for loc in slice_locs:
                # print("slice loc: " + str(px2rad(loc, wp, theta_fov_depth_horiz)))
                # get vertical slice (constant x)fr
                slice = depth_image[:,loc]

                # put into pairs [rad,depth]
                data = np.array([y_array, slice])

                # remove zeros
                mask = data[1,:] != 0
                data = data[:,mask]

                # if data is not empty from masking zeros, continue
                if data.shape[1] > 2:
                    # find slope (proper derivative)
                    slope = np.gradient(data[1,:], data[0,:]) # [mm/rad]

                    ##############
                    # find peaks #
                    ##############
                    peaks, _ = find_peaks(slope, height=100, distance=20, prominence=1000)
                    # peaks is index into slope - index 0 is top of image
                    # slope is 1D array deriv of data
                    # data is [rad, depth] pairs

                    if len(peaks) >= 2:
                        # we only want two peaks: the bottom and top edges of Obstacle
                        # therefore if there are extraneous peaks, we slice just the "first" two,
                        # first with respect to scanning from bottom to top
                        # since pixel coords go from top to bottom, we take
                        # the last two peaks instead of the first two
                        peaks = peaks[-2:]

                        # peaks[0] is the top edge index
                        # bump it down a bit to make sure it is on the obstacle face
                        # and not on the background
                        peaks[0] += 10
                        # bump bottom edge up a bit to make sure it is on face
                        # and not on foreground
                        peaks[1] -= 10

                        # mask data again
                        pitch_pair = data[0,peaks] # pitch coordinates, i.e. y-coord
                        dist_pair = data[1,peaks]  # distances found at given coords

                        face, theta = analyze_obstacle(pitch_pair[1], pitch_pair[0], dist_pair[1], dist_pair[0])
                        # print("{pa:.5f}, {pb:.5f} | {da:3.0f}, {db:3.0f} | {f:3.3f}".format(
                        #     pa = pitch_pair[1],
                        #     pb = pitch_pair[0],
                        #     da = dist_pair[1],
                        #     db = dist_pair[0],
                        #     f = face))

                        # print face length estimation on depth image
                        if test_case == 11:
                            plt.text(px2rad(loc, wp, theta_fov_depth_horiz), pitch_pair[0], f"f={face:3.1f}", color='cyan')
                        if test_case == 12:
                            plt.text(px2rad(loc, wp, theta_fov_depth_horiz), pitch_pair[0], f"f={face:3.1f},th={math.degrees(theta):4.1f}", color='cyan')

                        # print("face length = " + str(f))
                        # print("angle = " + str(math.degrees(theta)))

                        for peak in peaks:
                            # only plot  peaks that below threshold in fov
                            if data[0,peak] < 0.2:
                                # convert slice location to radian, this is the x-coord in plot we are looking at
                                # data[0,peak] is the radian value along the vertical slice that peak occurs at
                                plt.plot(px2rad(loc, wp, theta_fov_depth_horiz), data[0,peak], "cx")
            plt.show()
            # break
    elif test_case == 13:

        '''
        refer to notebook p95 for supporting work

        original implementation of analyze_obstacle() used law of SINES for theta3:
            theta3 = math.asin(d2/f*math.sin(alpha)) # law of sines

        but this implementation returned the supplement of theta3 for some orientations

        new implementation uses law of COSINES for theta3:
            theta3 = math.acos((d1**2+f**2-d2**2)/(2*d1*f)) # law of cosines

        this implementation, as tested below, works in all obstacle position cases
        '''

        ###########################
        # case 1: below eye level #
        ###########################
        print("############")
        print("#  CASE 1  #")
        print("############")

        t1,d1 = -32.72, 7.18

        A = [-30.76, 6.21]
        B = [-17.3, 6.33]
        C = [-15.41, 8.96]

        # expected values:
        # f, theta_obs
        expA = [1,45]
        expB = [2,90]
        expC = [3,150]
        expected = [expA,expB,expC]

        test_points = [A,B,C]

        print("error_f, error_theta_obs")
        for k,point in enumerate(test_points):
            f, theta_obs = tester(t1,d1,point[0],point[1])
            print("{f:.0f}, {t:.0f}".format(f=abs(f-expected[k][0]), t=abs(theta_obs-expected[k][1])))

        # print("f, theta_obs")
        # for point in test_points:
        #     f, theta_obs = tester(t1,d1,point[0],point[1])
        #     print("{f:.0f}, {t:.0f}".format(f=f, t=theta_obs))

        print()
        ##############################
        # case 2: spanning eye level #
        ##############################
        print("############")
        print("#  CASE 2  #")
        print("############")

        t1,d1 = -18.68, 2.71

        A = [28.80, 1.79]
        B = [23.76, 2.81]
        C = [14.95, 4.86]
        expected = [expA,expB,expC]

        # expected values:
        # f, theta_obs
        expA = [2,60]
        expB = [2,90]
        expC = [3,135]
        expected = [expA,expB,expC]

        test_points = [A,B,C]

        print("error_f, error_theta_obs")
        for k,point in enumerate(test_points):
            f, theta_obs = tester(t1,d1,point[0],point[1])
            print("{f:.0f}, {t:.0f}".format(f=abs(f-expected[k][0]), t=abs(theta_obs-expected[k][1])))

        # print("f, theta_obs")
        # for point in test_points:
        #     f, theta_obs = tester(t1,d1,point[0],point[1])
        #     print("{f:.0f}, {t:.0f}".format(f=f, t=theta_obs))

        print()
        ###########################
        # case 3: below eye level #
        ###########################
        print("############")
        print("#  CASE 3  #")
        print("############")

        t1,d1 = 5.09, 4.47

        A = [27.19, 3.06]
        B = [17.42, 4.66]
        C = [20.96, 7.04]

        # expected values:
        # f, theta_obs
        expA = [2,30]
        expB = [1,90]
        expC = [3,135]
        expected = [expA,expB,expC]

        test_points = [A,B,C]

        print("error_f, error_theta_obs")
        for k,point in enumerate(test_points):
            f, theta_obs = tester(t1,d1,point[0],point[1])
            print("{f:.0f}, {t:.0f}".format(f=abs(f-expected[k][0]), t=abs(theta_obs-expected[k][1])))

        # print("f, theta_obs")
        # for point in test_points:
        #     f, theta_obs = tester(t1,d1,point[0],point[1])
        #     print("{f:.0f}, {t:.0f}".format(f=f, t=theta_obs))
    elif test_case == 14:
        # get images
        image_pair = get_aligned_frame(pipeline)
        sleep(0.5)
        # capture a second time so auto exposure has a chance to activate
        image_pair = get_aligned_frame(pipeline)
        depth_image = image_pair[1]
        color_image = image_pair[0]

        plot_image_2d(depth_image)

        while True:
            image_pair = get_aligned_frame(pipeline)
            depth_image = image_pair[1]

            num_slices = 99
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
            # pitch and dist pairs are [bottom_edge, top edge]
            # yaw gives side-to-side location

            filtered_obstacles = filter_obstacles(
                obstacle_data,
                thresh_face_angle=math.radians(135),
                thresh_min_face_length=40,
                thresh_distance=1000,
                visualize=True
                )

            print("filtered_obstacles:")
            print(filtered_obstacles)

            plt.plot(obstacle_data[4],obstacle_data[0],'o',label='raw obstacles')
            plt.plot(filtered_obstacles[4],filtered_obstacles[0],'o',label='filtered obstacles')
            plt.legend()
            plt.title("Potential Obstacle Edges, num_slices={}".format(num_slices))
            plt.xlabel("yaw [rad]")
            plt.ylabel("face size [mm]")


            # ax.legend()
            plt.show()
    elif test_case == 15:
        # get images
        image_pair = get_aligned_frame(pipeline)
        sleep(0.5)
        # capture a second time so auto exposure has a chance to activate
        image_pair = get_aligned_frame(pipeline)
        depth_image = image_pair[1]
        color_image = image_pair[0]

        plot_image_2d(depth_image)

        while True:
            print("----- NEW SCAN -----")
            image_pair = get_aligned_frame(pipeline)
            depth_image = image_pair[1]

            num_slices = 50
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

            filtered_obstacles = filter_obstacles(
                obstacle_data,
                thresh_face_angle=math.radians(135),
                thresh_min_face_length=40,
                thresh_max_face_length=100,
                thresh_distance=1000,
                visualize=True
                )

            # print("filtered_obstacles:")
            # print(filtered_obstacles)

            fix, axs = plt.subplots(2,1)

            axs[0].plot(obstacle_data[4],obstacle_data[0],'o',label='raw obstacles')
            axs[0].plot(filtered_obstacles[4],filtered_obstacles[0],'o',label='filtered obstacles')
            axs[0].legend()
            # plt.title("Potential Obstacle Edges, num_slices={}".format(num_slices))
            axs[0].set_xlabel("yaw [rad]")
            axs[0].set_ylabel("face size [mm]")
            axs[0].set_xlim(-.6, 0.6)

            # now get gradient of face size
            slope = np.gradient(filtered_obstacles[0],filtered_obstacles[4])

            slope_thresh = 100
            slope_mask = abs(slope) < slope_thresh
            slope_filtered = slope[slope_mask]
            yaw_filtered = filtered_obstacles[4][slope_mask]

            axs[1].plot(filtered_obstacles[4],slope,'o')
            axs[1].plot(yaw_filtered,slope_filtered,'o')


            axs[1].set_xlim(-.6, 0.6)

            plt.show()
    elif test_case == 16:
        # get images
        image_pair = get_aligned_frame(pipeline)
        sleep(0.5)
        # capture a second time so auto exposure has a chance to activate
        image_pair = get_aligned_frame(pipeline)
        depth_image = image_pair[1]
        color_image = image_pair[0]

        plot_image_2d(depth_image)

        while True:
            print("----- NEW SCAN -----")
            image_pair = get_aligned_frame(pipeline)
            depth_image = image_pair[1]

            num_slices = 90
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

            filtered_obstacles = filter_obstacles(
                obstacle_data,
                thresh_face_angle=math.radians(135),
                thresh_min_face_length=40,
                thresh_max_face_length=150,
                thresh_distance=1000,
                visualize=True
                )

            # print("filtered_obstacles:")
            # print(filtered_obstacles)


            fix, axs = plt.subplots(3,2)

            ##########################
            # face size of obstacles #
            ##########################
            axs[0,0].plot(obstacle_data[4],obstacle_data[0],'o',label='raw obstacles')
            axs[0,0].plot(filtered_obstacles[4],filtered_obstacles[0],'o',label='filtered obstacles')
            axs[0,0].legend()
            # plt.title("Potential Obstacle Edges, num_slices={}".format(num_slices))
            axs[0,0].set_xlabel("yaw [rad]")
            axs[0,0].set_ylabel("face size [mm]")
            axs[0,0].set_xlim(-.6, 0.6)

            ##########################
            #  face angles #
            ##########################
            axs[0,1].plot(obstacle_data[4],[math.degrees(ang) for ang in obstacle_data[1]],'o',label='face angle')
            axs[0,1].set_xlabel("yaw [rad]")
            axs[0,1].set_ylabel("face angle [deg]")
            axs[0,1].set_xlim(-.6, 0.6)
            axs[0,1].legend()

            ##########################
            #  top and bottom edge distances #
            ##########################
            axs[1,0].plot(obstacle_data[4],[pair[0] for pair in obstacle_data[3]],'o',label='top edge dist')
            axs[1,0].plot(obstacle_data[4],[pair[1] for pair in obstacle_data[3]],'o',label='bot edge dist')

            axs[1,0].set_xlabel("yaw [rad]")
            axs[1,0].set_ylabel("dist [mm]")
            axs[1,0].set_xlim(-.6, 0.6)
            axs[1,0].legend()

            ##########################
            # dif b/t top/bot edge distances #
            ##########################
            top_edge_dist = [pair[0] for pair in obstacle_data[3]]
            bot_edge_dist = [pair[1] for pair in obstacle_data[3]]
            top_minus_bottom_edge_dist = [pair[0]-pair[1] for pair in obstacle_data[3]]

            axs[2,0].plot(obstacle_data[4], top_minus_bottom_edge_dist ,'o',label='top-bot edge dist')
            axs[2,0].set_xlabel("yaw [rad]")
            axs[2,0].set_ylabel("dist [mm]")
            axs[2,0].set_xlim(-.6, 0.6)
            axs[2,0].legend()

            ##########################
            #  top and bottom pitch angles #
            ##########################
            axs[1,1].plot(obstacle_data[4],[math.degrees(pair[0]) for pair in obstacle_data[2]],'o',label='top edge pitch')
            axs[1,1].plot(obstacle_data[4],[math.degrees(pair[1]) for pair in obstacle_data[2]],'o',label='bot edge pitch')
            axs[1,1].set_xlabel("yaw [rad]")
            axs[1,1].set_ylabel("pitch [deg]")
            axs[1,1].set_xlim(-.6, 0.6)
            axs[1,1].legend()

            ##########################
            #  dif b/t top/bot pitch angles #
            ##########################
            top_pitch_angle = [math.degrees(pair[0]) for pair in obstacle_data[2]]
            bot_pitch_angle = [math.degrees(pair[1]) for pair in obstacle_data[2]]
            top_minus_bottom_edge_angle = [math.degrees(pair[0] - pair[1]) for pair in obstacle_data[2]]

            axs[2,1].plot(obstacle_data[4], top_minus_bottom_edge_angle ,'o',label='top-bot edge pitch')
            axs[2,1].set_xlabel("yaw [rad]")
            axs[2,1].set_ylabel("pitch [deg]")
            axs[2,1].set_xlim(-.6, 0.6)
            axs[2,1].legend()

            ##########################
            #  #
            ##########################


            plt.show()
    elif test_case == 17:
        # get images
        image_pair = get_aligned_frame(pipeline)
        sleep(0.5)
        # capture a second time so auto exposure has a chance to activate
        image_pair = get_aligned_frame(pipeline)
        depth_image = image_pair[1]
        color_image = image_pair[0]

        plot_image_2d(depth_image)

        while True:
            print("----- NEW SCAN -----")
            image_pair = get_aligned_frame(pipeline)
            depth_image = image_pair[1]

            num_slices = 90
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

            filtered_obstacles = filter_obstacles(
                obstacle_data,
                thresh_face_angle=math.radians(135),
                thresh_min_face_length=40,
                thresh_max_face_length=150,
                thresh_distance=500,
                visualize=True
                )

            # print("filtered_obstacles:")
            # print(filtered_obstacles)


            fix, axs = plt.subplots(3,2)

            ##########################
            # face size of obstacles #
            ##########################
            axs[0,0].plot(obstacle_data[4],obstacle_data[0],'o',label='raw obstacles')
            axs[0,0].plot(filtered_obstacles[4],filtered_obstacles[0],'o',label='filtered obstacles')
            axs[0,0].legend()
            # plt.title("Potential Obstacle Edges, num_slices={}".format(num_slices))
            axs[0,0].set_xlabel("yaw [rad]")
            axs[0,0].set_ylabel("face size [mm]")
            axs[0,0].set_xlim(-.6, 0.6)

            ##########################
            #  face angles #
            ##########################
            axs[0,1].plot(filtered_obstacles[4],np.degrees(filtered_obstacles[1]),'o',label='face angle')

            axs[0,1].set_xlabel("yaw [rad]")
            axs[0,1].set_ylabel("face angle [deg]")
            axs[0,1].set_xlim(-.6, 0.6)
            axs[0,1].legend()

            ##########################
            #  top and bottom edge distances #
            ##########################
            axs[1,0].plot(filtered_obstacles[4],filtered_obstacles[3][:,0],'o',label='top edge dist')
            axs[1,0].plot(filtered_obstacles[4],filtered_obstacles[3][:,1],'o',label='bot edge dist')

            axs[1,0].set_xlabel("yaw [rad]")
            axs[1,0].set_ylabel("dist [mm]")
            axs[1,0].set_xlim(-.6, 0.6)
            axs[1,0].legend()

            ##########################
            # dif b/t top/bot edge distances #
            ##########################
            top_minus_bottom_edge_dist = filtered_obstacles[3][:,0]-filtered_obstacles[3][:,1]
            axs[2,0].plot(filtered_obstacles[4], top_minus_bottom_edge_dist ,'o',label='top-bot edge dist')

            axs[2,0].set_xlabel("yaw [rad]")
            axs[2,0].set_ylabel("dif_dist [mm]")
            axs[2,0].set_xlim(-.6, 0.6)
            axs[2,0].legend()

            ##########################
            #  top and bottom pitch angles #
            ##########################
            axs[1,1].plot(filtered_obstacles[4],filtered_obstacles[2][:,0] ,'o',label='top edge pitch')
            axs[1,1].plot(filtered_obstacles[4],filtered_obstacles[2][:,1] ,'o',label='bot edge pitch')

            axs[1,1].set_xlabel("yaw [rad]")
            axs[1,1].set_ylabel("pitch [deg]")
            axs[1,1].set_xlim(-.6, 0.6)
            axs[1,1].legend()

            ##########################
            #  dif b/t top/bot pitch angles #
            ##########################
            top_minus_bottom_edge_angle = np.degrees(filtered_obstacles[2][:,0]-filtered_obstacles[2][:,1])
            axs[2,1].plot(filtered_obstacles[4], top_minus_bottom_edge_angle ,'o',label='top-bot edge pitch')

            axs[2,1].set_xlabel("yaw [rad]")
            axs[2,1].set_ylabel("dif_pitch [deg]")
            axs[2,1].set_xlim(-.6, 0.6)
            axs[2,1].legend()

            ##########################
            #  #
            ##########################


            plt.show()
    elif test_case == 18:
        # get images
        image_pair = get_aligned_frame(pipeline)
        sleep(0.5)
        # capture a second time so auto exposure has a chance to activate
        image_pair = get_aligned_frame(pipeline)
        depth_image = image_pair[1]
        color_image = image_pair[0]

        plot_image_2d(depth_image)

        while True:
            print("----- NEW SCAN -----")
            image_pair = get_aligned_frame(pipeline)
            depth_image = image_pair[1]

            num_slices = 90
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

            filtered_obstacles = filter_obstacles(
                obstacle_data,
                thresh_face_angle=math.radians(135),
                thresh_min_face_length=40,
                thresh_max_face_length=150,
                thresh_distance=500,
                visualize=True
                )


            ##########################
            # face size of obstacles #
            ##########################
            plt.plot(obstacle_data[4],obstacle_data[0],'o',label='raw obstacles',markersize=3)
            plt.plot(filtered_obstacles[4],filtered_obstacles[0],'o',label='filtered obstacles',markersize=5)

            try:
                obstacle_yaw_bounds = [filtered_obstacles[4][0], filtered_obstacles[4][-1]]
                plt.plot(filtered_obstacles[4][0],filtered_obstacles[0][0],'r>',label='left bound',markersize=15,mec='k')
                plt.plot(filtered_obstacles[4][-1],filtered_obstacles[0][-1],'r<',label='right bound',markersize=15,mec='k')

            except:
                print("no bounds found")

            plt.plot()


            plt.legend(loc='upper right')
            plt.title("Obstacle Detection, num_slices={}".format(num_slices))
            plt.xlabel("yaw [rad]")
            plt.ylabel("face size [mm]")
            plt.xlim(-.6, 0.6)

            plt.show()
    elif test_case == 19:
        # get images
        image_pair = get_aligned_frame(pipeline)
        sleep(0.5)
        # capture a second time so auto exposure has a chance to activate
        image_pair = get_aligned_frame(pipeline)
        depth_image = image_pair[1]
        color_image = image_pair[0]

        plot_image_2d(depth_image)

        while True:
            # print("----- NEW SCAN -----")
            image_pair = get_aligned_frame(pipeline)
            depth_image = image_pair[1]

            num_slices = 90
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
                filtered_obstacles = filter_obstacles(
                    obstacle_data,
                    thresh_face_angle=math.radians(135),
                    thresh_min_face_length=40,
                    thresh_max_face_length=150,
                    thresh_distance=500,
                    visualize=True
                    )

                obstacle_yaw_bounds = [filtered_obstacles[4][0], filtered_obstacles[4][-1]]
                obstacle_center = np.average(obstacle_yaw_bounds)

                if obstacle_center > 0:
                    turn_dir = "LEFT      "
                elif obstacle_center < 0:
                    turn_dir = "     RIGHT"
                else:
                    turn_dir = "x"

                print("Obstacle center: {c:6.2f}, TURN {dir:s} to avoid".format(c=math.degrees(obstacle_center), dir=turn_dir))
            except:
                print("empty array somewhere...")


            # if not obstacle_data:
            #     print("No edges detected.")
            # else:
            #     print("filtering potential obstacles")
            #     filtered_obstacles = filter_obstacles(
            #         obstacle_data,
            #         thresh_face_angle=math.radians(135),
            #         thresh_min_face_length=40,
            #         thresh_max_face_length=150,
            #         thresh_distance=500,
            #         visualize=True
            #         )
            #     print("filtered")
            #
            #     if not filtered_obstacles:
            #         print("No obstacles detected after filtering.")
            #     else:
            #         print("filtered obs detected")
            #         obstacle_yaw_bounds = [filtered_obstacles[4][0], filtered_obstacles[4][-1]]
            #         obstacle_center = np.average(obstacle_yaw_bounds)
            #
            #         if obstacle_center > 0:
            #             turn_dir = "LEFT      "
            #         elif obstacle_center < 0:
            #             turn_dir = "     RIGHT"
            #         else:
            #             turn_dir = "x"
            #
            #         print("Obstacle center: {c:6.2f}, TURN {dir:s} to avoid".format(c=math.degrees(obstacle_center), dir=turn_dir))

    # elif test_case ==
    # elif test_case ==
    # elif test_case ==
    # elif test_case ==
    # elif test_case ==
    # elif test_case ==
    # elif test_case ==
    # elif test_case ==
    # elif test_case ==
    # elif test_case ==
    # elif test_case ==

    # Stop streaming
    pipeline.stop()
    print("Stream stopped")


if __name__ == "__main__":
    main()
