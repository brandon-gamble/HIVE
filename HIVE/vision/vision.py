import pyrealsense2.pyrealsense2 as rs
import numpy as np 
import cv2 
import argparse
import imutils
import sys

def detect_aruco(image_pair, cloud_param, visualize=False):
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
    print("looking for markers")
    (corners, ids, rejected) = cv2.aruco.detectMarkers(color_image, arucoDict, parameters=arucoParams)
    

    #######################    
    ## visualize         ##
    #######################
    if visualize is True:
        
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
            d = cloud.sum()/(cloud!=0).sum()
            
            # compute heading (pixel) 
            heading_p = cX - image_center_x

            markers.append([markerID,[cX,cY],d,heading_p])

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
                print("Marker ID: {id:<3} Loc: ({x:<3},{y:<3}) Dist [mm]: {d:.2f} Heading [pxl]: {h:<3}".format(
                    id = markerID,
                    x = cX,
                    y = cY, 
                    d = d,
                    h = heading_p))
                
                # show image
                cv2.imshow("image",color_image)
                cv2.waitKey(0)
            # end of visualization routine 
            
    else:
        print("no markers detected")
    
    if visualize is True:
        print("all markers displayed") 
        cv2.waitKey(0) 

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

def main():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    image_pair = get_curr_frame(pipeline)
    markers = detect_aruco(image_pair, (10, 2), visualize=True)

    for marker in markers:
        print("Marker ID: {id:<3} Loc: ({x:<3},{y:<3}) Dist [mm]: {d:.2f} Heading [pxl]: {h:<3}".format(
            id = marker[0],
            x = marker[1][0],
            y = marker[1][1], 
            d = marker[2],
            h = marker[3]))

    # Stop streaming
    pipeline.stop()


if __name__ == "__main__":
    main()
