'''
to test if system id and control system were successful.

takes a list of control inputs over time and sends them to arduino.

TO USE:
    1) using follow_path.py, generate a txt file of actuation commands
    2) using this script, load in actuation txt file
    3) plug into arduino and let it rip! see if tank follows path in simulation
'''



import numpy as np
import sys
import time

sys.path.insert(1, 'D:/all my files/documents/uvm/5_masters/hive/github_directory/HIVE/motion/python/send_serial_motor_control')
from send_actuation_msg import *

#   1   mimic HIVE/controller_sim/follow_path.py example

if __name__ == '__main__':

    test = 1

    if test == 1:
        # this test uses a 20 ms controller step (noted in txt file and can be found in original follow_path test)
        actuation_data = np.loadtxt('follow_path_test05_actuatorCommands.txt', dtype=int)
        T_phys = 0.001
        T_cont = 0.020
        ratio = int(T_cont / T_phys)

        step = 3
        n = step*ratio
        actuation_data = actuation_data[::n,:] # slice out every nth point

        # build array of command messages
        k = actuation_data.shape[0]
        msg_array = []
        for cmd in actuation_data:
            msg  = '<L,' + str(cmd[0]) + '>'
            msg += '<R,' + str(cmd[1]) + '>'
            msg_array.append(msg)


        # initialize connection
        ser = initialize_com(38400);

        # wait for connection
        print("waiting for handshake...")
        time.sleep(5)

        # start sending commands
        k = 0
        print('SENDING COMMANDS')
        for msg in msg_array:
            # send message to arduino
            send_msg(ser,msg)

            # print to console
            #print(str(k) + msg)

            # wait for controller discretization step time before sending another command
            time.sleep(T_cont*step)

            k += 1
        print('COMPLETE')
