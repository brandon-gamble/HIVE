import pyrealsense2.pyrealsense2 as rs
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import cm
import cv2
import argparse
import imutils
import sys

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

def plot_image_2d(image):
    # "image" is array, not frame

    # good cmap options: inferno, hsv
    plt.imshow(image, cmap='inferno', vmin=0, vmax=1500)
    plt.colorbar()
    plt.title('Realsense FOV, Depth [mm]')
    plt.xlabel('x [px]')
    plt.ylabel('z [px]')
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
    03  converts
    '''
    test_case = 3

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
            a_px = 150
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

            plt.xlabel('x [deg]')
            plt.ylabel('depth [mm]')
            plt.legend()
            plt.title('Realsense FOV depth slice [mm]')
            plt.show()

    # Stop streaming
    pipeline.stop()
    print("Stream stopped")


if __name__ == "__main__":
    main()
