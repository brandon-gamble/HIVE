from tkinter import *
from scipy.interpolate import interp1d as map
import time

lib_path = "D:/all my files/documents/uvm/5_masters/hive/github_directory/HIVE/motion/python/send_serial_motor_control"
import sys
sys.path.append(lib_path)
import send_actuation_msg as commander

############################
#   establish connection   #
############################
ser = commander.initialize_com(38400);
print('waiting for connection....')
time.sleep(5)
print('connected....')
print('***************************')

###################
#   make window   #
###################
# create window/frame
win= Tk()
# set size of frame
dim_int = 250
dim_str = str(dim_int)
# create argument in form "dimxdim" e.g. "250x250"
geom_str = dim_str+"x"+dim_str
# pass to geometry() to set frame size
win.geometry(geom_str)

# callback for getting x,y loc of mouse
def callback(e):
    # px,py pixel locations
    px= e.x
    py= e.y

    max_s = 175 # max base speed
    max_w = 175 # max base omega

    # define maps #
    # s assoc with y
    map_s = map([dim_int,0],[-max_s,max_s])
    # w assoc with x
    map_w = map([0,dim_int],[max_w,-max_w])

    # compute mapping
    s = map_s(py)
    w = map_w(px)

    # compute motor commands, bounded -255 to +255 (motor command bounds)
    m_l = max(-255, min(255,s-w))
    m_r = max(-255, min(255,s+w))

    # format commands into serial message
    msg = '<L,%d><R,%d>'%(m_l,m_r)
    # send message to motors
    commander.send_msg(ser,msg)

    # print("Pointer is currently at %d, %d" %(px,py))
    # print("s: %d w: %d" % (s,w))
    print("m_l: %d m_r: %d" % (m_l,m_r))
    return

def disable(e):
    commander.send_msg(ser,"<S,1>")
    print("MOTORS DISABLED")
    return

def enable(e):
    commander.send_msg(ser,"<S,0>")
    print("MOTORS ENABLED")
    return

win.bind('<Motion>',callback)
win.bind('<Button-1>',enable)
win.bind('<Button-3>',disable)
#win.pack()
win.mainloop()
