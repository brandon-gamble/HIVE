import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def extract_python_controller_data(path, header_loc, column_of_interest):
    df = pd.read_csv(path, header=header_loc, sep=',')
    array = df.values

    time = array[:,0]
    data = array[:,column_of_interest]

    return [time, data]

gain_strings = ['01','02','04','06','08','10']

# small angle
######################################################
small_dic = {}
for gain in gain_strings:
    path = 'head_small/24-11-25_head-gain-test_small_' + gain + 'x00_00.csv'
    small_dic[gain]=extract_python_controller_data(path, 21, 2)

    my_label = 'Kh=' + str(int(gain))
    plt.plot(small_dic[gain][0],np.degrees(small_dic[gain][1]),label=my_label)

plt.xlim([0.8,5])
plt.ylim([-10,30])
plt.xlabel('Time [s]')
plt.ylabel('Heading [deg]')
plt.title('Head gain, small step 10deg')
plt.legend()
plt.show()

# large angle
######################################################
large_dic = {}
for gain in gain_strings:
    path = 'head_large/24-11-25_head-gain-test_large_' + gain + 'x00_00.csv'
    large_dic[gain]=extract_python_controller_data(path, 21, 2)

    my_label = 'Kh=' + str(int(gain))
    plt.plot(large_dic[gain][0],large_dic[gain][1]*180/3.14,label=my_label)

plt.xlim([0.8,5])
plt.ylim([-10, 60])
plt.xlabel('Time [s]')
plt.ylabel('Heading [deg]')
plt.title('Head gain, large step 40deg')
plt.legend()
plt.show()

# side by side
#################################################

this_dic = small_dic
plt.figure(figsize=(11,4.8))
plt.subplot(1,2,1)
for key in this_dic:
    my_label = 'Kh=' + str(int(key))
    plt.plot(this_dic[key][0],np.degrees(this_dic[key][1]),label=my_label)
plt.xlim([0.8,5])
plt.ylim([-10, 60])
plt.xlabel('Time [s]')
plt.ylabel('Heading [deg]')
plt.title('Small Step (10 deg)')
plt.legend()

this_dic = large_dic
plt.subplot(1,2,2)
for key in this_dic:
    my_label = 'Kh=' + str(int(key))
    plt.plot(this_dic[key][0],this_dic[key][1]*180/3.14,label=my_label)
plt.xlim([0.8,5])
plt.ylim([-10, 60])
plt.xlabel('Time [s]')
plt.title('Large Step (40 deg)')
plt.legend()

plt.suptitle('Heading Gain')
plt.show()

plt.xlim([0.8,10])
plt.ylim([-10,60])
plt.xlabel('Time [s]')
plt.ylabel('Heading [rad]')
plt.title('Head gain, small and large steps')
plt.show()
