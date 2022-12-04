import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

#######################
# STOCK DRIVER SIGNAL #
#######################
date = '2022-11-30'
lo = 0
hi = 8

period = 0.00319        # period of signal
resolution = 0.00001    # sample time
samples_per_wave = int(period/resolution) # how many samples we need to plot to get one waveform (plus some buffer)
leading_padding = 10 # how many samples to start before rising edge

# where waveform starts (found with _analysis.py)
starting_indices = [354,260,245,923,1147,976,666,354,355]

plt.subplot(2,1,1)
plt.xlabel('time [us]')
plt.ylabel('voltage [V]')
plt.title('Stock Driver')
#plt.xlim([0,3150])
#plt.ylim([-2,11])

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

    # slice out one wave with a bit of padding
    first = starting_indices[m]-leading_padding
    last = starting_indices[m] + samples_per_wave
    volt = volt[first:last]

    time_s = np.arange(0,len(volt))*resolution # time vector to plot against
    time_us = time_s * 1000000

    plt.plot(time_us, 10*volt)


# plt.show()



####################
# TB6612FNG SIGNAL #
####################
date = '2022-11-30'
lo = 9
hi = 22

period = 0.00204        # period of signal
resolution = 0.000005    # sample time
samples_per_wave = int(period/resolution) # how many samples we need to plot to get one waveform (plus some buffer)
leading_padding = 5 # how many samples to start before rising edge

# where waveform starts (found with _analysis.py)
starting_indices = [648,649,299,1001,935,1004,1005,320,378,445,369,376,692,659]

plt.subplot(2,1,2)
plt.xlabel('time [us]')
plt.ylabel('voltage [V]')
plt.title('TB6612FNG Driver')
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

    # slice out one wave with a bit of padding
    first = starting_indices[m]-leading_padding
    last = starting_indices[m] + samples_per_wave
    #volt = volt[first:last]
    volt = volt[first:-1]

    time_s = np.arange(0,len(volt))*resolution # time vector to plot against
    time_us = time_s * 1000000

    plt.plot(time_us, 10*volt)

plt.show()
