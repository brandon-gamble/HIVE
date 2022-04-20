from tkinter import *
from scipy.interpolate import interp1d as map
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

    max_s = 100 # max base speed
    max_w = 100 # max base omega

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

    # print("Pointer is currently at %d, %d" %(px,py))
    # print("s: %d w: %d" % (s,w))
    print("m_l: %d m_r: %d" % (m_l,m_r))

win.bind('<Motion>',callback)
win.mainloop()
