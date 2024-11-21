import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import imutils
import sys

def detect_aruco(image_pair, visualize=False, printout=False):
    #######################
    ## def, load, detect ##
    #######################

    markers = []

    # define aruco dictionary
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)

    # get parameters
    arucoParams = cv2.aruco.DetectorParameters()

    # split images
    color_image = image_pair[0]
    depth_image = image_pair[1]

    # get image width and center
    image_width = int(color_image.shape[1])
    image_center_x = int(image_width/2)

    # detect aruco
    #print("looking for markers")
    #(corners, ids, rejected) = cv2.aruco.detectMarkers(color_image, arucoDict, parameters=arucoParams)
    mydetector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    (corners, ids, rejected) = mydetector.detectMarkers(color_image)


    #######################
    ## visualize         ##
    #######################
    if visualize is True:

        # show color_image
        cv2.imshow("image",color_image)
        cv2.waitKey(0)

    if len(corners) > 0: # i.e. at least 1 marker detected

        # flatten list
        ids = ids.flatten()

        for (markerCorner, markerID) in zip(corners, ids):
            # extract corners (returned in
            # top-left, top-right, bot-right, bot-left order)
            corners = markerCorner.reshape((4,2))
            (topLeft, topRight, botRight, botLeft) = corners

            # convert x,y pairs to integers
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            topRight = (int(topRight[0]), int(topRight[1]))
            botRight = (int(botRight[0]), int(botRight[1]))
            botLeft = (int(botLeft[0]), int(botLeft[1]))

            # compute center
            cX = int((topLeft[0] + botRight[0]) / 2.0)
            cY = int((topLeft[1] + botRight[1]) / 2.0)

            # get distance
            # note TRANSPOSITION of X and Y
            d = depth_image[cY, cX]

            # compute heading (pixel)
            heading_p = cX - image_center_x

            # markers.append([markerID,cX,cY,d,heading_p])
            markers.append([markerID,cX,cY,d,heading_p,topLeft[0],topLeft[1],topRight[0],topRight[1],botRight[0],botRight[1],botLeft[0],botLeft[1]])
            # 0     markerID,
            # 1,2   cX,cY,
            # 3,4   d,heading_p,
            # 5,6   topLeft[0],topLeft[1],
            # 7,8   topRight[0],topRight[1],
            # 9,10  botRight[0],botRight[1],
            # 11,12 botLeft[0],botLeft[1]

            if visualize is True:
                # draw bounding box
                cv2.line(color_image, topLeft, topRight, (0,255,0), 2)
                cv2.line(color_image, topRight, botRight, (0,255,0), 2)
                cv2.line(color_image, botRight, botLeft, (0,255,0), 2)
                cv2.line(color_image, botLeft, topLeft, (0,255,0), 2)

                # draw center circle
                cv2.circle(color_image, (cX, cY), 4, (0,0,255), -1)

                # draw marker id
                cv2.putText(color_image, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0,255,0), 2)

                # show image
                cv2.imshow("image",color_image)
                cv2.waitKey(0)

            # end of visualization routine

            if printout is True:
                # print out
                print("{id:<3} ({x:3},{y:3}) {d:10.2f} {h:10}".format(
                    id = markerID,
                    x = cX,
                    y = cY,
                    d = d,
                    h = heading_p))



    # else:
    #     print("no markers detected")

    if printout is True:
        print("all markers displayed")
        cv2.waitKey(0)

    return markers

def detect_aruco_cloudAvg(image_pair, cloud_param, visualize=False):
    # when computing distance, we take the median value of
    # an array size NxN centered at cX,cY
    # where N=cloud_size and cX,cY is the center of the detected aruco marker
    cloud_size = cloud_param[0]
    cloud_step = cloud_param[1]

    #######################
    ## def, load, detect ##
    #######################

    markers = []

    # define aruco dictionary
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)

    # get parameters
    arucoParams = cv2.aruco.DetectorParameters()

    # split images
    color_image = image_pair[0]
    depth_image = image_pair[1]

    # get image width and center
    image_width = int(color_image.shape[1])
    image_center_x = int(image_width/2)

    # detect aruco
    #print("looking for markers")
    #(corners, ids, rejected) = cv2.aruco.detectMarkers(color_image, arucoDict, parameters=arucoParams)
    mydetector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)
    (corners, ids, rejected) = mydetector.detectMarkers(color_image)


    #######################
    ## visualize         ##
    #######################
    if visualize is True:

        # show color_image
        cv2.imshow("image",color_image)
        cv2.waitKey(0)

    if len(corners) > 0: # i.e. at least 1 marker detected

        # flatten list
        ids = ids.flatten()

        for (markerCorner, markerID) in zip(corners, ids):
            # extract corners (returned in
            # top-left, top-right, bot-right, bot-left order)
            corners = markerCorner.reshape((4,2))
            (topLeft, topRight, botRight, botLeft) = corners

            # convert x,y pairs to integers
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            topRight = (int(topRight[0]), int(topRight[1]))
            botRight = (int(botRight[0]), int(botRight[1]))
            botLeft = (int(botLeft[0]), int(botLeft[1]))

            # compute center
            cX = int((topLeft[0] + botRight[0]) / 2.0)
            cY = int((topLeft[1] + botRight[1]) / 2.0)

            # want to make a small cloud around center
            # and find an average distance

            # make cloud bounds
            yMin = cY - cloud_size
            yMax = cY + cloud_size
            xMin = cX - cloud_size
            xMax = cX + cloud_size

            # NOTE THE TRANSPOSITION OF X AND Y
            cloud = depth_image[yMin:yMax:cloud_step, xMin:xMax:cloud_step]

            # # get median value of cloud
            # d = np.median(cloud)

            # get average, ignoring zeros
            try:
                d = cloud.sum()/(cloud!=0).sum()
            except:
                d = 0

            # compute heading (pixel)
            heading_p = cX - image_center_x

            markers.append([markerID,cX,cY,d,heading_p])

            if visualize is True:
                # draw bounding box
                cv2.line(color_image, topLeft, topRight, (0,255,0), 2)
                cv2.line(color_image, topRight, botRight, (0,255,0), 2)
                cv2.line(color_image, botRight, botLeft, (0,255,0), 2)
                cv2.line(color_image, botLeft, topLeft, (0,255,0), 2)

                # draw center circle
                cv2.circle(color_image, (cX, cY), 4, (0,0,255), -1)

                # draw marker id
                cv2.putText(color_image, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0,255,0), 2)

                # print out
                print("{id:<3} ({x:3},{y:3}) {d:10.2f} {h:10}".format(
                    id = markerID,
                    x = cX,
                    y = cY,
                    d = d,
                    h = heading_p))

                # show image
                cv2.imshow("image",color_image)
                cv2.waitKey(0)
            # end of visualization routine

    # else:
    #     print("no markers detected")

    if visualize is True:
        print("all markers displayed")
        cv2.waitKey(0)

    return markers

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

def approx_dist(image_pair, theta_fov, marker_size, marker):

    color_image = image_pair[0]
    image_width = int(color_image.shape[1])
    image_center_x = int(image_width/2)

    yp = image_width / (2*np.tan(theta_fov/2))

    theta = np.arctan(abs(marker[1]-image_center_x)/yp)

    # calculate side lengths
    x1 = marker[5]
    y1 = marker[6]
    x2 = marker[7]
    y2 = marker[8]
    marker_size_px_top = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    x1 = marker[7]
    y1 = marker[8]
    x2 = marker[9]
    y2 = marker[10]
    marker_size_px_right = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    x1 = marker[9]
    y1 = marker[10]
    x2 = marker[11]
    y2 = marker[12]
    marker_size_px_bot = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    x1 = marker[11]
    y1 = marker[12]
    x2 = marker[5]
    y2 = marker[6]
    marker_size_px_left = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    marker_size_px_avg = (marker_size_px_top+marker_size_px_right+marker_size_px_bot+marker_size_px_left)/4

    y = marker_size*yp/marker_size_px_avg

    dist = y/np.cos(theta)

    return dist

def approx_dist_simp(image_pair, theta_fov, marker_size, marker):
    # NOTE: marker size INCLUDES padding, because detection algorithm finds box around padding

    color_image = image_pair[0]
    image_width = int(color_image.shape[1])
    image_center_x = int(image_width/2)

    yp = image_width / (2*np.tan(theta_fov/2))

    # calculate side lengths
    x1 = marker[5]
    y1 = marker[6]
    x2 = marker[7]
    y2 = marker[8]
    marker_size_px_top = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    x1 = marker[7]
    y1 = marker[8]
    x2 = marker[9]
    y2 = marker[10]
    marker_size_px_right = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    x1 = marker[9]
    y1 = marker[10]
    x2 = marker[11]
    y2 = marker[12]
    marker_size_px_bot = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    x1 = marker[11]
    y1 = marker[12]
    x2 = marker[5]
    y2 = marker[6]
    marker_size_px_left = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    marker_size_px_avg = (marker_size_px_top+marker_size_px_right+marker_size_px_bot+marker_size_px_left)/4

    xp = abs(marker[1]-image_center_x)

    dist = marker_size*yp/marker_size_px_avg*((1+(xp/yp)**2)**0.5)
    # dist = marker_size/marker_size_px_avg*((xp**2+yp**2)**0.5)

    return dist

def main():
    '''
    1   original implementation
    2   "simplified" equation compared to original - should be EQUAL
    '''
    test_case = 1

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    print("ID | Loc [px] | Dist [mm] | ~Dist [mm] | Error_d [%] | Heading [px]")
    print("-------------------------------------------------------------------")


    while True:
        if test_case == 1:
            # marker_size = 37.5 # mm
            marker_size = 38.5 # mm

            image_pair = get_aligned_frame(pipeline)
            markers = detect_aruco(image_pair, visualize=False)

            for marker in markers:
                rough_dist = approx_dist(image_pair, 1.518, marker_size, marker)

                print("{id:<3} ({x:3},{y:3}) {d:10.2f} {d_approx:11.2f} {error:14.2f} {h:10}".format(
                    id = marker[0],
                    x = marker[1],
                    y = marker[2],
                    d = marker[3],
                    d_approx = rough_dist,
                    error = 100*abs(rough_dist-marker[3])/marker[3],
                    h = marker[4]))

            # input()

        if test_case == 2:
            # marker_size = 37.5 # mm
            marker_size = 38.5 # mm

            image_pair = get_aligned_frame(pipeline)
            markers = detect_aruco(image_pair, visualize=False)

            for marker in markers:
                rough_dist = approx_dist(image_pair, 1.518, marker_size, marker)
                rough_dist_2 = approx_dist_simp(image_pair, 1.518, marker_size, marker)

                print("{d1:10.2f} {d2:10.2f} {e:5.2f}".format(
                    d1 = rough_dist,
                    d2 = rough_dist_2,
                    e = abs(rough_dist-rough_dist_2)))

            # input()

    # Stop streaming
    pipeline.stop()
    print("Stream stopped")


if __name__ == "__main__":
    main()
