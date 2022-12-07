import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

#######################
# STOCK DRIVER SIGNAL #
#######################
date = '2022-11-30'
lo = 0
hi = 8

leading_padding = 10 # how many samples to start before rising edge

# where waveform starts (found with _analysis.py)
starting_indices = [354,260,245,923,1147,976,666,354,355]

plt.subplot(2,1,1)
# plt.xlabel('time [us]')
plt.ylabel('voltage [V]')
plt.title('Stock Driver [313 Hz]')
plt.xlim([0,3150])
plt.ylim([-2,11])

m = -1
for k in range(lo,hi+1):
    m+=1 # keep track of which starting indice to use

    # build filename
    filename= date + '_' + str(k).zfill(2) + '.csv'

    # read data
    df = pd.read_csv(filename, skiprows=5)
    df.columns=['id','time','volt1','volt2','na']

    # convert data to numpy array
    volt = df.volt1.to_numpy()
    time_s = df.time.to_numpy()

    # slice out one wave with a bit of padding
    first = starting_indices[m]-leading_padding
    volt = volt[first:-1]
    time_s = time_s[first:-1]

    # offset so that starting time is 0
    time_s = time_s - time_s[0]
    time_us = time_s * 1000000

    if k == 0:
        plt.plot(time_us, 10*volt)
    if k == 8:
        plt.plot(time_us, 10*volt)
    if k == 4:
        plt.plot(time_us, 10*volt)
    if k == 2:
        plt.plot(time_us, 10*volt)



# plt.show()



####################
# TB6612FNG SIGNAL #
####################
date = '2022-11-30'
lo = 9
hi = 22

leading_padding = 5 # how many samples to start before rising edge

# where waveform starts (found with _analysis.py)
starting_indices = [648,649,299,1001,935,1004,1005,320,378,445,369,376,692,659]

plt.subplot(2,1,2)
plt.xlabel('time [us]')
plt.ylabel('voltage [V]')
plt.title('TB6612FNG Driver [490 Hz]')
plt.xlim([0,3150])
plt.ylim([-2,11])

m = -1
for k in range(lo,hi+1):
    m+=1 # keep track of which starting indice to use

    # build filename
    filename= date + '_' + str(k).zfill(2) + '.csv'

    # read data
    df = pd.read_csv(filename, skiprows=5)
    df.columns=['id','time','volt1','volt2','na']

    # convert data to numpy array
    volt = df.volt1.to_numpy()
    time_s = df.time.to_numpy()

    # slice out one wave with a bit of padding
    first = starting_indices[m]-leading_padding
    volt = volt[first:-1]
    time_s = time_s[first:-1]

    # offset so that starting time is 0
    time_s = time_s - time_s[0]
    time_us = time_s * 1000000

    if k == 9:
        plt.plot(time_us, 10*volt)
    if k == 15:
        plt.plot(time_us, 10*volt)
    if k == 20:
        plt.plot(time_us, 10*volt)
    if k == 22:
        plt.plot(time_us, 10*volt)

plt.show()
