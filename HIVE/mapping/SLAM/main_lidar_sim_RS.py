import env, sensors
import pygame
import math
# import random
import pyrealsense2.pyrealsense2 as rs
import matplotlib.pyplot as plt
import numpy as np
import cv2
import argparse
import imutils
import sys

sys.path.append('D:/all my files/documents/uvm/5_masters/hive/github_directory/HIVE/mapping/')
import mapping

def rs_sensor_data(depth_image, position):
    #################################################
    # get realsense data

    # set camera specs
    wp = 640
    theta_fov_depth = math.radians(87)

    # convert horizontal pixels to headings
    x_array = np.arange(wp)
    x_array = mapping.px2rad(x_array, wp, theta_fov_depth)

    # set slice locations
    a_px = 240

    # get slices
    z_slice_a = mapping.get_z_slice(depth_image, a_px)

    # put into pairs [rad, depth]
    data_a = np.array([x_array,z_slice_a])

    # remove zeros from each set
    mask_a = data_a[1,:] != 0
    data_a = data_a[:, mask_a]

    # plot depth image
    #mapping.plot_image_2d(depth_image)
    # plot color image
    #plt.imshow(color_image)
    #plt.show()

    #################################################

    # put realsense data into "environment format"
    sensor_data = []
    for k in range(max(np.shape(data_a))):
        # flip angle array bc of pixel origin
        # rotate by pi/2   bc of pixel origin
        # divide dist by 2 to fit in window better
        sensor_data.append([data_a[1,k]/2, data_a[0,-k]+3.14/2, (position[0], position[1])])
        #                     dist       angle       x             y

        # only flipped angle
        #sensor_data.append([data_a[1,k], data_a[0,-k], (position[0], position[1])])

    return sensor_data


def main():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    environment = env.buildEnvironment((600, 1200))
    environment.originalmap = environment.map.copy()
    laser = sensors.LaserSensor(200, environment.originalmap, uncertainty=(0.5, 0.01))
    environment.map.fill((0, 0, 0))
    environment.infomap = environment.map.copy()

    running = True

    while running:
        sensor_on = False

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if pygame.mouse.get_focused():
                sensor_on = True
            elif not pygame.mouse.get_focused():
                sensor_on = False

        if sensor_on:
            position = pygame.mouse.get_pos()
            laser.position = position

            image_pair = mapping.get_aligned_frame(pipeline)
            depth_image = image_pair[1]
            color_image = image_pair[0]

            sensor_data = rs_sensor_data(depth_image, position)

            environment.data_storage(sensor_data)
            environment.show_sensor_data()


            '''
            position = pygame.mouse.get_pos()
            laser.position = position
            sensor_data = laser.sense_obstacles()
            environment.data_storage(sensor_data)
            environment.show_sensor_data()
            '''

        environment.map.blit(environment.infomap, (0, 0))

        pygame.display.update()

if __name__ == "__main__":
    main()
