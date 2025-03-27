import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

###############
# attenuation #
###############

syd_id_df = pd.read_csv('24-09-20_sys_id_data.csv',header=4,sep=',')

#   0      1        2      3
# time  command  omegaL omegaR
sys_id_array = syd_id_df.values
time = sys_id_array[:,0]
cmd = sys_id_array[:,1]

# attenutiaon is control val / omega
atten_L = cmd/sys_id_array[:,2]
atten_R = cmd/sys_id_array[:,3]

plt.plot(time, atten_L, label='Attenuation left')
plt.plot(time, atten_R, label='Attenuation right')
#plt.plot([5,15],[8,8])

plt.xlabel('Time [s]')
plt.ylabel('Attenuation [s/rad]')
plt.title('Control Signal Attenuation')
plt.ylim([0,20])
plt.xlim([5,15])
plt.legend(loc='upper left')
plt.show()

############
# response #
############
from scipy import signal
num = [2.896]
den = [1, 24.35]
system = signal.TransferFunction(num,den)
t,y = signal.step(system,T=time)

y50=y*50
y100=y*100
y200=y*200

plt.subplot(1,2,1)
plt.plot(time, sys_id_array[:,2],label='Experimental')
plt.plot(time+4,y50,label='Model, amp=50')
plt.plot(time+8,y100,label='Model, amp=100')
plt.plot(time+12,y200,label='Model, amp=200')
# plt.plot(time,composite,label='Model')
# plt.plot(t,y*50,label='Model')
plt.xlabel("Time [s]")
plt.ylabel("Omega [rad/s]")
plt.title("Left Motor")
plt.legend(loc='upper left')
plt.xlim([0, 15])
plt.ylim([0,30])

####################################################################
num = [3.143]
den = [1, 24.94]
system = signal.TransferFunction(num,den)
t,y = signal.step(system,T=time)

y50=y*50
y100=y*100
y200=y*200

plt.subplot(1,2,2)
plt.plot(time, sys_id_array[:,3],label='Experimental')
plt.plot(time+4,y50,label='Model, amp=50')
plt.plot(time+8,y100,label='Model, amp=100')
plt.plot(time+12,y200,label='Model, amp=200')
plt.xlabel("Time [s]")
plt.ylabel("Omega [rad/s]")
plt.title("Right Motor")
plt.legend(loc='upper left')
plt.xlim([0, 15])
plt.ylim([0,30])

# plt.plot(time, sys_id_array[:,1],label='Command value')
plt.suptitle("Motor Step Response Experimental vs Model")
plt.show()
