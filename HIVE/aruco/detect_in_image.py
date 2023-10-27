import argparse
import imutils
import cv2
import sys

    #####################
    # IMPORTANT LESSONS # 
    #####################
    # - markers can reliably be identified even when skewed and small
    # - marker doesn't have to be black, but DOES need to be consistent
    #     color throughout
    # - marker needs to have a border. border can be thin and any color.
    
def detect_aruco_static(image_path):
    #######################
    ## def, load, detect ##
    #######################

    # define aruco dictionary
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    
    # load image
    print("loading image [{}] ...".format(image_path))
    image = cv2.imread(image_path)
    # if image is large and markers are small, then may need to adjust resize value
    image = imutils.resize(image, width=600)
    
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
            print("Aruco marker ID: {}".format(markerID))
            
            # show image
            cv2.imshow("image",image)
            cv2.waitKey(0)
            
    else:
        print("no markers detected")
    
    print("all markers displayed") 
    cv2.waitKey(0)      
            
def main():

    image_paths = []

    ################
    # test block 1 # 
    ################
    # big, high res markers on white background, some skewed
    # results: full detection
   
    image_paths.append("detection_images/multi_tag.png")
    image_paths.append("detection_images/multi_tag_x1.png")
        # note that one marker doesn't get identified in x1
        # this is because it has been reflected -> it is not 
        # supposed to be identified bc it is no longer a valid marker


    ################
    # test block 2 #
    ################
    # small, lower res markers. mixed background (cityscape, white, bordered)
    # results:
    #   jpg and png both work
    #   markers not detected without white border

    image_paths.append("detection_images/city.jpg")
        # none detected
    image_paths.append("detection_images/city_white.png")
        #       all detected
    image_paths.append("detection_images/city_white_border.png")       
        #       all detected     
    image_paths.append("detection_images/city_big.png")
        # none detected
    image_paths.append("detection_images/city_big_white.jpg")
        #       all detected 


    ################
    # test block 3 #
    ################
    # constraints: size and color of border
    # results: 
        # cannot detect gray marker
        # very thin border is detectable when a bright color
        # very thin border fails when dark (nearly black)
    
    image_paths.append("detection_images/city_borders_colors.png")
        # found all but 2: 
            # top L: made darkspace of marker gray
            # bot R: very thin and dark border


    ################
    # test block 4 #
    ################
    # constraints: detectable marker color? (non-black)
    # results: all detected. seems that failure to detect in 
    #   city_border_colors.png was because the squares in the 
    #   center of the marker were not recolored to gray. therefore
    #   marker contained 2 colors.
    
    image_paths.append("detection_images/colors.png")



    #############
    # RUN TESTS # 
    #############
    
    for path in image_paths:
        detect_aruco_static(path);
    
if __name__ == "__main__":
    main()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
