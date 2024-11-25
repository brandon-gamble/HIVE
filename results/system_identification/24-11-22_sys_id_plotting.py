import pandas as pd
import matplotlib.pyplot as plt

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
plt.title('Control signal attenuation')
plt.ylim([0,20])
plt.xlim([5,15])
plt.legend(loc='upper left')
plt.show()

############
# response #
############

plt.plot(time, sys_id_array[:,2],label='Response, Left')
plt.plot(time, sys_id_array[:,3],label='Response, Right')
plt.plot(time, sys_id_array[:,1],label='Command value')
plt.legend(loc='upper left')
plt.show()
