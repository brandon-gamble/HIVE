import numpy as np
import matplotlib.pyplot as plt

t = [0,3,5,10,15,20,25,30,35,40,45,50,55,60,65]
M_025 = [25,70.1,82.64,91,98.4,98,98.7,99.5,99.5,100.4,98.7]
M_050 = [50,67.6,108.1,135.1,150.8,159.6,165.9,174.5,179.9,187.7,196.5,200.8,208,208.7,207.8,207.5]
M_075 = ['75-STALL',79.5,115.8,140,181,220.2,0]  # STALL AT 15
M_100 = ['100-STALL',79.1,137.8,177,0]  # STALL AT 6
M_125 = ['125-STALL',80.9,141.0,181,0]  # STALL AT 5
M_150 = ['150-STALL',79.7,150,195.2,0]  # STALL AT 5
M_175 = ['175-STALL',78.9,151,182,211.6,0]  # STALL AT 10
M_200 = ['200-STALL',78.9,131,141,155,165,188.8,193.7,191.1,213.8,226.5,222.9,221,0]  # STALL AT 50
M_225 = [225,81.3,101.7,115.3,125,134,137.6,138.3,151.3,154,154,145,142,143.4]
M_250 = [250,81.3,89.3,94.6,98,98,101.6,101.8,99.5,103.6,102.9,103.2,102.7,104.7,107.2]

data = [M_025, M_050, M_075, M_100, M_125, M_150, M_175, M_200, M_225, M_250]

#############################################
# temp vs time (for each motor command)
for test in data:
    n = len(test)-1
    if 0 in test:
        plt.plot(t[0:n],test[1:], '--', label=str(test[0]))
    else:
        plt.plot(t[0:n],test[1:], '-', label=str(test[0]))
plt.title("TB6612FNG Overheat Test")
plt.xlabel("Time [s]")
plt.ylabel("Temp [F]")
plt.legend(loc="lower right")
plt.show()

#############################################
# temp increase vs motor command (after 3 seconds)
cmd = [25,50,75,100,125,150,175,200,225,250]
tmp = []
for test in data:
    tmp.append(test[2]-test[1])
plt.plot(cmd,tmp)
plt.xlabel('Motor Command')
plt.ylabel('Temperature Change [F]')
plt.title('Temperature Change vs Motor Command at 3 seconds')
plt.show()

#############################################
# temp increase vs motor command (after 3 and 5 seconds)
cmd = [25,50,75,100,125,150,175,200,225,250]
tmp3 = []
for test in data:
    tmp3.append(test[2]-test[1])
plt.plot(cmd,tmp3, label='3s elapsed')
tmp5 = []
for test in data:
    tmp5.append(test[3]-test[1])
plt.plot(cmd,tmp5, label='5s elapsed')
plt.legend(loc='upper right')
plt.xlabel('Motor Command')
plt.ylabel('Temperature Change [F]')
plt.title('Temperature Change vs Motor Command')
plt.show()

#############################################
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
for test in data:
    temp = test[1:]
    n = len(temp)
    time = t[0:n]
    motor = list(np.ones([n])*test[0])

    motor_time_temp = np.zeros([n,3])
    motor_time_temp[:,0] = motor
    motor_time_temp[:,1] = time
    motor_time_temp[:,2] = temp

    ax.scatter(motor,time,temp)
ax.set_xlabel('motor command')
ax.set_ylabel('time')
ax.set_zlabel('temp')
plt.show()
