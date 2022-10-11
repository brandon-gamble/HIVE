'''
to test if system id and control system were successful.

takes a list of control inputs over time and sends them to arduino.


'''



import numpy as np
import sys
import time

sys.path.insert(1, 'D:/all my files/documents/uvm/5_masters/hive/github_directory/HIVE/motion/python/send_serial_motor_control')
from send_actuation_msg import *

#   1   mimic HIVE/controller_sim/follow_path.py example

test = 1

if test == 1:
    # this test uses a 20 ms controller step (noted in txt file and can be found in original follow_path test)
    actuation_data = np.loadtxt('follow_path_test05_actuatorCommands.txt', dtype=int)
    T_cont = 0.02

    # initialize connection
    ser = initialize_com(38400);

    # wait for connection
    print("waiting for handshake...")
    time.sleep(5)

    # start sending commands
    step = 0
    for cmd in actuation_data:
        # build message to send
        msg  = '<L,' + str(cmd[0]) + '>'
        msg += '<R,' + str(cmd[1]) + '>'

        # send message to arduino
        send_msg(ser,msg)

        # print to console
        print(str(step) + msg)

        # wait for controller discretization step time before sending another command
        time.sleep(T_cont)

        step += 1
