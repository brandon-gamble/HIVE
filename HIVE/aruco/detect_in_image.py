import argparse
import imutils
import cv2
import sys

def main():
#    image_path = "detection_images/multi_tag.png"
    image_path = "detection_images/multi_tag_x1.png"
        # note that one marker doesn't get identified in x1
        # this is because it has been reflected -> it is not 
        # supposed to be identified bc it is no longer a valid marker

    #######################
    ## def, load, detect ##
    #######################

    # define aruco dictionary
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    
    # load image
    print("loading image...")
    image = cv2.imread(image_path)
    # if image is large and markers are small, then may need to adjust resize value
    image = imutils.resize(image, width=600)
    
    # show image
    cv2.imshow("image",image)
    cv2.waitKey(0)
    
    # get parameters     
#    arucoParams = cv2.aruco.DetectorParameters_create()
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
            print("Aruco marker ID: {}".format(markerID))
            
            # show image
            cv2.imshow("image",image)
            cv2.waitKey(0)
    else:
        print("no markers detected")
                   
            
    
    
if __name__ == "__main__":
    main()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
