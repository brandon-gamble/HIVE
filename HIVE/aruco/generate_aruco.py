import cv2

# https://www.makeuseof.com/python-aruco-marker-generator-how-create/

def generate_single(marker_size,marker_id):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
    
    cv2.imwrite("marker_{}.png".format(marker_id), marker_img)

    marker_img = cv2.imread("marker_{}.png".format(marker_id))

    cv2.imshow("Marker", marker_img)

    cv2.waitKey(1000)
   
def main():
    generate_single(400,30)
    for k in range(0,10):
        generate_single(400,k)

if __name__ == "__main__":
    main()
