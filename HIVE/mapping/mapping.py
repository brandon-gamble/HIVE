import pyrealsense2.pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import cv2
import argparse
import imutils
import sys

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

    ############################################
    # print(depth_frame.get_distance(10,10))
    # print(depth_image[10,10])
    ############################################

    # # Stack into RGBD image
    # RGBD_image = np.dstack((color_image,depth_image))

    frame_pair = (color_frame, depth_frame)
    image_pair = (color_image, depth_image)

    return image_pair

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

def plot_image(image):
    # "image" is array, not frame

    height = image.shape[0]
    width = image.shape[1]

    x = np.arange(0,width)
    y = np.arange(0,height)
    X, Y = np.meshgrid(x, y)

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

    surf = ax.plot_surface(X, Y, image, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)
    
    
    plt.show()

    return

def main():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    while True:
        #ui = input("new frame?")
        # if ui == "plot":
            # plt.show()
        image_pair = get_curr_frame(pipeline)
        depth_image = image_pair[1]

        z_slice_120 = get_z_slice(depth_image, 120, 30)
        z_slice_240 = get_z_slice(depth_image, 240, 30)
        z_slice_360 = get_z_slice(depth_image, 360, 30)

        # plt.plot(z_slice_120)
        # plt.plot(z_slice_240)
        # plt.plot(z_slice_360)

        plot_image(depth_image)





        

    # Stop streaming
    pipeline.stop()
    print("Stream stopped")


if __name__ == "__main__":
    main()
