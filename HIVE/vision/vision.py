import pyrealsense2.pyrealsense2 as rs
import numpy as np 
import cv2 
import argparse
import imutils
import sys

def detect_aruco(image_pair, cloud_param):
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

    print("color and depth shapes")
    print(color_image.shape)
    print(depth_image.shape)
    
    # detect aruco 
    print("looking for markers")
    (corners, ids, rejected) = cv2.aruco.detectMarkers(color_image, arucoDict, parameters=arucoParams)
    

    #######################    
    ## visualize         ##
    #######################
    if True:
        
        # show color_image
        cv2.imshow("image",color_image)
        cv2.waitKey(0)
        
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
                cv2.line(color_image, topLeft, topRight, (0,255,0), 2)   
                cv2.line(color_image, topRight, botRight, (0,255,0), 2)
                cv2.line(color_image, botRight, botLeft, (0,255,0), 2)
                cv2.line(color_image, botLeft, topLeft, (0,255,0), 2)  
                
                # compute center and draw
                cX = int((topLeft[0] + botRight[0]) / 2.0)
                cY = int((topLeft[1] + botRight[1]) / 2.0)            
                cv2.circle(color_image, (cX, cY), 4, (0,0,255), -1)

                # find distance to center of marker 
                # NOTE THE TRANSPOSITION OF X AND Y 
                yMin = cY - cloud_size
                yMax = cY + cloud_size
                xMin = cX - cloud_size
                xMax = cX + cloud_size
                
                cloud = depth_image[yMin:yMax:cloud_step, xMin:xMax:cloud_step]

                d = np.median(cloud)
                
                # draw marker id
                cv2.putText(color_image, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0,255,0), 2)

                # print("Aruco marker ID: {id:<3} Location: ({x:<3},{y:<3})".format(
                #     id = markerID,
                #     x = cX,
                #     y = cY)) 

                print("Aruco marker ID: {id:<3} Location: ({x:<3},{y:<3}) Distance [mm]: {d:<5}".format(
                    id = markerID,
                    x = cX,
                    y = cY, 
                    d = d))

                # span = 4
                # step = 10
                # yLo = cY - span*step
                # yHi = cY + span*step
                # xLo = cX - span*step
                # xHi = cX + span*step
                # print(depth_image[yLo:yHi:step,xLo:xHi:step])

                markers.append([markerID,[cX,cY],d])
                
                # show image
                cv2.imshow("image",color_image)
                cv2.waitKey(0)
                
        else:
            print("no markers detected")
        
        print("all markers displayed") 
        cv2.waitKey(0) 

    # end of visualization routine

    return markers

def get_curr_frame(pipeline):
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    # Convert images to numpy arrays    
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # # Stack into RGBD image
    # RGBD_image = np.dstack((color_image,depth_image))

    image_pair = (color_image, depth_image)

    return image_pair


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

image_pair = get_curr_frame(pipeline)
markers = detect_aruco(image_pair, (1, 1))

# for marker in markers:
#     print("Aruco marker ID: {id:<3} Location: ({x:<3},{y:<3}) Distance: {d:<5}".format(
#         id = marker[0],
#         x = marker[1,0],
#         y = marker[1,1], 
#         d = marker))

# Stop streaming
pipeline.stop()