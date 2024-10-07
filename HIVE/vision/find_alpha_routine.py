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

    return dist

def marker_pxl_size_angle(image_pair, marker):

    # calculate side lengths
    x1 = marker[5]
    y1 = marker[6]
    x2 = marker[7]
    y2 = marker[8]
    x3 = marker[9]
    y3 = marker[10]
    x4 = marker[11]
    y4 = marker[12]

    marker_size_px_top   = np.sqrt((x2-x1)**2 + (y2-y1)**2)
    marker_size_px_right = np.sqrt((x3-x2)**2 + (y3-y2)**2)
    marker_size_px_bot   = np.sqrt((x4-x3)**2 + (y4-y3)**2)
    marker_size_px_left  = np.sqrt((x1-x4)**2 + (y1-y4)**2)

    marker_size_px_avg = (marker_size_px_top+marker_size_px_right+marker_size_px_bot+marker_size_px_left)/4

    dx = x2 - x1
    dy = y2 - y1
    angle_rad = np.arctan2(dy,dx)

    return marker_size_px_avg, angle_rad

def main():
    marker_size = 30 # mm
    marker_size = 50 # mm
    N = 5 # bit size of marker (NOT INCLUDING ANY PADDING)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    print("----------------------------------------------------------------------------------")
    print("N [bit], Size [mm], Size [px], Dist [mm], Angle [rad], Angle [deg], Alpha [px/bit]")
    print("----------------------------------------------------------------------------------")

    while True:

        image_pair = get_aligned_frame(pipeline)
        markers = detect_aruco(image_pair, visualize=False)

        for marker in markers:
            rough_dist = approx_dist_simp(image_pair, 1.518, marker_size, marker)
            marker_size_px, angle_rad = marker_pxl_size_angle(image_pair, marker)
            # marker size px includes the first black padding,
            # so add 2 bits to N, the actual marker bits
            alpha = marker_size_px / (N+2) 

            print("{N:7},{m_mm:10.1f},{m_px:10.2f},{d:10.2f},{theta:12.5f},{thetad:12.3f},{alpha:15.3f}".format(
                N = N,
                m_mm = marker_size,
                m_px = marker_size_px,
                d = rough_dist,
                theta = angle_rad,
                thetad = angle_rad*180/3.14159,
                alpha = alpha
                ))

if __name__ == "__main__":
    main()
