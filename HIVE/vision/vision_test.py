import pyrealsense2.pyrealsense2 as rs
import numpy as np 
import cv2 

import argparse
import imutils
import sys

def detect_aruco_static(image):
    #######################
    ## def, load, detect ##
    #######################

    markers = []

    # define aruco dictionary
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    
    # show image
    cv2.imshow("image",image)
    cv2.waitKey(0)
    
    # get parameters     
    arucoParams = cv2.aruco.DetectorParameters()
    
    # detect aruco 
    print("looking for markers")
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    

    #######################    
    ## visualize         ##
    #######################
    
    if len(corners) > 0: # i.e. at least 1 marker detected
        print("markers detected")
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
            
            # draw ####################
            
            # draw bounding box
            cv2.line(image, topLeft, topRight, (0,255,0), 2)   
            cv2.line(image, topRight, botRight, (0,255,0), 2)
            cv2.line(image, botRight, botLeft, (0,255,0), 2)
            cv2.line(image, botLeft, topLeft, (0,255,0), 2)  
            
            # compute center and draw
            cX = int((topLeft[0] + botRight[0]) / 2.0)
            cY = int((topLeft[1] + botRight[1]) / 2.0)            
            cv2.circle(image, (cX, cY), 4, (0,0,255), -1)
            
            # draw marker id
            cv2.putText(image, str(markerID),
                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0,255,0), 2)


            print("Aruco marker ID: {id:<3} Location: ({x:<3},{y:<3})".format(id = markerID,x = cX,y = cY))
            markers.append([markerID,cX,cY])
            
            # show image
            cv2.imshow("image",image)
            cv2.waitKey(0)
            
    else:
        print("no markers detected")
    
    print("all markers displayed") 
    cv2.waitKey(0) 

    return markers



#DEBUG cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    # while True:
    kill=1
    while kill != 0:
        kill=0 

        
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays    
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        
        scaled_depth=cv2.convertScaleAbs(depth_image, alpha=0.08)
        depth_colormap = cv2.applyColorMap(scaled_depth, cv2.COLORMAP_JET)


        # Stack both images horizontally
        #DEBUG images = np.hstack((color_image, depth_colormap))

        # Show images
        #DEBUG cv2.imshow('RealSense', images)

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break


    markers = detect_aruco_static(color_image)
    print(markers)


finally:

    # Stop streaming
    pipeline.stop()