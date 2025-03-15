import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# def extract_python_controller_data(path, header_loc, column_of_interest):
#     df = pd.read_csv(path, header=header_loc, sep=',')
#     array = df.values
#
#     time = array[:,0]
#     data = array[:,column_of_interest]
#
#     return [time, data]

test_ids = ['05','08','09','11','12']
base_name = '25-03-12_gtg-'
# 0 Time Elapsed [s],
# 1 Dist [mm],
# 2 Head [rad],
# 3 s_des [m/s],
# 4 omega_des [rad/s],
# 5 omega_l_des [rad/s],
# 6 omega_r_des [rad/s]

plt.figure(figsize=(6,4))

time_shift_dic = {'05':8.735,'08':10.91,'09':7.124,'11':7.709,'12':8.212}

################################################################################
# extract data, mask it, and plot dist
################################################################################
data_dic = {}
k=1 # to make sequential trial ids
for test in test_ids:
    # build path string
    path = base_name + test + '.csv'
    # extract data into dataframe within dictionary
    data_dic[test] = pd.read_csv(path, header=17, sep=',')

    # apply time shift to line up data to same starting point
    data_dic[test].iloc[:,0] -= time_shift_dic[test]

    # filter out times when marker is not detected
    t_lo = 0
    if test == '12':
        # test 12 lost marker during test so need to filter
        # larger time range
        t_hi = 17
    else:
        t_hi = 2
    # for compound bool
    t_lo_bool = data_dic[test]['Time Elapsed [s]'] > t_lo
    t_hi_bool = data_dic[test]['Time Elapsed [s]'] < t_hi
    dist_bool = data_dic[test][' Dist [mm]'] == -500
    # this mask is where mark is not detected (combined bool)
    marker_found = t_lo_bool & t_hi_bool & dist_bool
    # invert mask to good data
    no_marker_found = ~marker_found
    # apply masks:
    # 1) shift the -500 dist values to 0
    data_dic[test][' Dist [mm]'][dist_bool]+=500
    # 2) remove points where marker was not detected
    data_dic[test] = data_dic[test][no_marker_found]


    plt.plot(data_dic[test]['Time Elapsed [s]'],
             data_dic[test][' Dist [mm]'],
             label='Test ID '+str(k))
             # label='Test ID '+test)
    k+=1

plt.legend()
plt.xlim([-5,30])
plt.xlabel('Time [s]')
plt.ylabel('Dist [mm]')
plt.title('GTG Tracking Tests')
plt.show()

################################################################################
# individual plots of head/speed/omega
################################################################################
my_cols = [' Head [rad]', ' s_des [m/s]', ' omega_des [rad/s]']

for col in my_cols:
    plt.figure(figsize=(6,4))
    k=1 # for sequential trial ids
    for key in data_dic.keys():
        plt.plot(data_dic[key]['Time Elapsed [s]'],
            data_dic[key][col],
            label='Test ID '+str(k))
            # label='Test ID '+key)
        k+=1
    plt.xlim([-5,30])
    plt.xlabel('Time [s]')
    plt.ylabel(col)
    plt.title('GTG Tracking Tests')
    plt.legend()
    plt.show()
# for key in data_dic:
#     print(data_dic[key].columns)

################################################################################
# dist and speed
################################################################################
plt.figure(figsize=(12,4))

############################################
# distance
############################################
plt.subplot(1,2,1)
k=1
for key in data_dic.keys():
    plt.plot(data_dic[key]['Time Elapsed [s]'],
        data_dic[key][' Dist [mm]'],
        label='Test ID '+str(k))
        # label='Test ID '+key)
    k+=1
plt.xlim([-5,30])
plt.xlabel('Time [s]')
plt.ylabel('Distance [mm]')
plt.legend()
# plt.title('GTG Tracking Tests')

############################################
# s_des
############################################
plt.subplot(1,2,2)
k=1
for key in data_dic.keys():
    plt.plot(data_dic[key]['Time Elapsed [s]'],
        data_dic[key][' s_des [m/s]'],
        label='Test ID '+str(k))
        # label='Test ID '+key)
    k+=1
plt.xlim([-5,30])
plt.xlabel('Time [s]')
plt.ylabel('S_des [m/s]')
plt.legend()
# plt.title('GTG Tracking Tests')

plt.suptitle('GTG Tracking Tests')
plt.show()

################################################################################
# heading and omega_des plots
################################################################################
plt.figure(figsize=(12,4))

############################################
# heading
############################################
plt.subplot(1,2,1)
k=1
for key in data_dic.keys():
    plt.plot(data_dic[key]['Time Elapsed [s]'],
        data_dic[key][' Head [rad]']*180/3.14,
        label='Test ID '+str(k))
        # label='Test ID '+key)
    k+=1
plt.xlim([-5,30])
plt.xlabel('Time [s]')
plt.ylabel('Heading [deg]')
plt.legend()
# plt.title('GTG Tracking Tests')

############################################
# omega_des
############################################
plt.subplot(1,2,2)
k=1
for key in data_dic.keys():
    plt.plot(data_dic[key]['Time Elapsed [s]'],
        data_dic[key][' omega_des [rad/s]']*180/3.14,
        label='Test ID '+str(k))
        # label='Test ID '+key)
    k+=1
plt.xlim([-5,30])
plt.xlabel('Time [s]')
plt.ylabel('Omega_des [deg/s]')
plt.legend()
# plt.title('GTG Tracking Tests')

plt.suptitle('GTG Tracking Tests')
plt.show()

################################################################################
# dist and heading (setpoint tracking)
################################################################################
plt.figure(figsize=(12,4))

############################################
# distance
############################################
plt.subplot(1,2,1)
k=1
for key in data_dic.keys():
    plt.plot(data_dic[key]['Time Elapsed [s]'],
        data_dic[key][' Dist [mm]'],
        label='Test ID '+str(k))
        # label='Test ID '+key)
    k+=1
plt.xlim([-5,30])
plt.xlabel('Time [s]')
plt.ylabel('Distance [mm]')
plt.legend()
# plt.title('GTG Tracking Tests')

############################################
# heading
############################################
plt.subplot(1,2,2)
k=1
for key in data_dic.keys():
    plt.plot(data_dic[key]['Time Elapsed [s]'],
        data_dic[key][' Head [rad]']*180/3.14,
        label='Test ID '+str(k))
        # label='Test ID '+key)
    k+=1
plt.xlim([-5,30])
plt.xlabel('Time [s]')
plt.ylabel('Heading [deg]')
plt.legend()
# plt.title('GTG Tracking Tests')

plt.suptitle('GTG Tracking Tests: Setpoint Tracking')
plt.show()

################################################################################
# actuation effort
################################################################################
plt.figure(figsize=(12,4))

############################################
# s_des
############################################
plt.subplot(1,2,1)
k=1
for key in data_dic.keys():
    plt.plot(data_dic[key]['Time Elapsed [s]'],
        data_dic[key][' s_des [m/s]'],
        label='Test ID '+str(k))
        # label='Test ID '+key)
    k+=1
plt.xlim([-5,30])
plt.xlabel('Time [s]')
plt.ylabel('S_des [m/s]')
plt.legend()
# plt.title('GTG Tracking Tests')

############################################
# omega_des
############################################
plt.subplot(1,2,2)
k=1
for key in data_dic.keys():
    plt.plot(data_dic[key]['Time Elapsed [s]'],
        data_dic[key][' omega_des [rad/s]'],
        label='Test ID '+str(k))
        # label='Test ID '+key)
    k+=1
plt.xlim([-5,30])
plt.xlabel('Time [s]')
plt.ylabel('Omega_des [rad/s]')
plt.legend()
# plt.title('GTG Tracking Tests')

plt.suptitle('GTG Tracking Tests: Actuation Effort')
plt.show()
