import pyrealsense2.pyrealsense2 as rs
import numpy as np
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

def get_z_slice(depth_image, slice_height, slice_size):
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
    plt.imshow(image, cmap='hsv')
    plt.colorbar()
    plt.show()
    return

def main():
    '''
    test files
    ---------------------------
    01  plot color map of depth image, then aligned color image.
        note that color image has smaller FOV, so when aligned has large black border
    02  plot 2d color map, 3d color map, then plot slices from map
    '''
    test_case = 2

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

            plot_image_2d(depth_image)
            plot_image_3d(depth_image)

            a = 200
            b = 370
            c = 420

            z_slice_a = get_z_slice(depth_image, a, 2)
            z_slice_b = get_z_slice(depth_image, b, 2)
            z_slice_c = get_z_slice(depth_image, c, 2)

            plt.plot(z_slice_a, label=str(a))
            plt.plot(z_slice_b, label=str(b))
            plt.plot(z_slice_c, label=str(c))

            plt.legend()
            plt.show()






    # Stop streaming
    pipeline.stop()
    print("Stream stopped")


if __name__ == "__main__":
    main()
