import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2

def print_cols(my_list, num_cols, col_width):
    #parent_methods = ['hi', 'hello', 'potato', 'one', 'twooasdf']
    i = 0 
    for x in range(int(len(my_list)/num_cols)+1):
        combined = ""
        for col in range(num_cols):
            try:
                combined += my_list[i].ljust(col_width)
            except: combined += ""
            i += 1
        print(combined)

#print("------------ RS ------------")
#print_cols(dir(rs), 4, 40)
# config 
# motion_frame
# motion_sensor
# motion_stream
# pipeline
# pose
# pose_frame
# pose_sensor
# pose_stream
# stream
# quaternion

#print("------------ RS.CONFIG ------------")
#print_cols(dir(rs.config), 4, 25)

#print("------------ rs.config.enable_stream ------------")
#print_cols(dir(rs.config.enable_stream), 4, 25) 

#print("------------ RS.STREAM ------------")
#print_cols(dir(rs.stream), 4, 25)
# depth
# color
# pose
# gyro

#print_cols(dir(rs.config.enable_stream), 1, 25)
#print_cols(dir(rs.pose_frame),1,25)

'''
# map of functions #
- rs
    - config
        - enable_stream
    - motion_frame
    - motion_sensor
    - motion_stream
    - pipeline
        - wait_for_frames
    - pose
        - acceleration
        - translation
        - velocity
    - pose_frame
        - get_data
        - pose_data
    - pose_sensor
    - pose_stream
    - stream
        > https://intelrealsense.github.io/librealsense/doxygen/rs__sensor_8h.html#a01b4027af33139de861408872dd11b93
            - ctrl F -> "Streams are different types of data provided by RealSense devices."
        - depth
        - color
        - pose -> internet says you can't access pose on D435i, only on Txxx model.
        - gyro
    - quaternion

frames = pipeline.wait_for_frames()
frame functions
    - get_color_frame
        - get_data()
            - np.asanyarray()
    - get_depth_frame
    - get_pose_frame
        - get_pose_data()
            - data.translation / velocity / acceleration
    - get_data

'''
'''
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#config.enable_stream(rs.stream.pose)
pipeline.start(config)

'''

'''
test cases
1   rgb frame
2
'''

test = 2

if test == 1:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)

    frames =      pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_data =  color_frame.get_data()
    color_image = np.asanyarray(color_data)

    
    cv2.imshow("image",color_image)

elif test == 2:
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.gyro) # able to enable gyro stream on D435i
    #config.enable_stream(rs.stream.pose) # can't enable pose stream on D435i
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    pipeline.start(config)

    frames = pipeline.wait_for_frames()

    print(frames[0].is_depth_frame())
    print(frames[1].is_video_frame())
    print(frames[2].is_motion_frame())

    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    pose_frame = frames.get_pose_frame() # -> THIS FRAME IS EMPTY
    # POSE FRAME EMPTY...

    



