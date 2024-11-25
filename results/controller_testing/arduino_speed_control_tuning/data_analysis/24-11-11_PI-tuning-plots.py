import pandas as pd
import matplotlib.pyplot as plt

##################################################################
#                            phase 1                             #
##################################################################

phase1_df = pd.read_csv('24-11-11_PI-tune_PHASE-1_I0_P-varied.csv',
                        header=1,
                        sep=',')

phase1_array = phase1_df.values
time = phase1_array[:,0]
omega_des = phase1_array[:,1]
kp_4 = phase1_array[:,2]
kp_6 = phase1_array[:,3]
kp_8 = phase1_array[:,4]
kp_12 = phase1_array[:,5]
kp_16 = phase1_array[:,6]

# single initial plot
#####################

plt.plot(time,omega_des,label='Setpoint')
plt.plot(time,kp_4,label='KP=4')

plt.xlim([0,25])
plt.ylim([0,30])
plt.xlabel('Time [s]')
plt.ylabel('Motor Speed [rad/s]')
plt.title('Phase 1: Proportional Tuning')
plt.legend(loc='upper left')
plt.show()


# overlay plot
#####################

plt.plot(time,omega_des,label='Setpoint')
plt.plot(time,kp_4,label='KP=4')
plt.plot(time,kp_6,label='KP=6')
plt.plot(time,kp_8,label='KP=8')
plt.plot(time,kp_12,label='KP=12')
#plt.plot(time,kp_16,label='KP=16')

plt.xlim([0,25])
plt.ylim([-10,30])
plt.xlabel('Time [s]')
plt.ylabel('Motor Speed [rad/s]')
plt.title('Phase 1: Proportional Tuning')
plt.legend()
plt.show()

# subplot
####################

fig, axs = plt.subplots(2,2,figsize=(9,7))
fig.suptitle("Phase 1: Proportional Tuning")

axs[0,0].plot(time,omega_des,label='Setpoint')
axs[0,0].plot(time,kp_4,label='KP=4')
axs[0,0].set_xlim([0,25])
axs[0,0].set_ylim([-10,30])
#axs[0,0].set_xlabel('Time [s]')
axs[0,0].set_ylabel('Motor Speed [rad/s]')
# axs[0,0].set_title('KP=4')
axs[0,0].legend(loc="upper left")

axs[0,1].plot(time,omega_des,label='Setpoint')
axs[0,1].plot(time,kp_6,label='KP=6')
axs[0,1].set_xlim([0,25])
axs[0,1].set_ylim([-10,30])
#axs[0,1].set_xlabel('Time [s]')
#axs[0,1].set_ylabel('Motor Speed [rad/s]')
# axs[0,1].set_title('KP=6')
axs[0,1].legend(loc="upper left")

axs[1,0].plot(time,omega_des,label='Setpoint')
axs[1,0].plot(time,kp_8,label='KP=8')
axs[1,0].set_xlim([0,25])
axs[1,0].set_ylim([-10,30])
axs[1,0].set_xlabel('Time [s]')
axs[1,0].set_ylabel('Motor Speed [rad/s]')
# axs[1,0].set_title('KP=8')
axs[1,0].legend(loc="upper left")

axs[1,1].plot(time,omega_des,label='Setpoint')
axs[1,1].plot(time,kp_12,label='KP=12')
axs[1,1].set_xlim([0,25])
axs[1,1].set_ylim([-10,30])
axs[1,1].set_xlabel('Time [s]')
#axs[1,1].set_ylabel('Motor Speed [rad/s]')
# axs[1,1].set_title('KP=12')
axs[1,1].legend(loc="upper left")
plt.show()

# subplot
####################

fig, axs = plt.subplots(2,2,figsize=(9,7))
fig.suptitle("Phase 1: Proportional Tuning")

axs[0,0].plot(time,omega_des,label='Setpoint')
axs[0,0].plot(time,kp_6,label='KP=6')
axs[0,0].set_xlim([0,25])
axs[0,0].set_ylim([-10,30])
#axs[0,0].set_xlabel('Time [s]')
axs[0,0].set_ylabel('Motor Speed [rad/s]')
# axs[0,0].set_title('KP=6')
axs[0,0].legend(loc="upper left")

axs[0,1].plot(time,omega_des,label='Setpoint')
axs[0,1].plot(time,kp_8,label='KP=8')
axs[0,1].set_xlim([0,25])
axs[0,1].set_ylim([-10,30])
#axs[0,1].set_xlabel('Time [s]')
#axs[0,1].set_ylabel('Motor Speed [rad/s]')
# axs[0,1].set_title('KP=8')
axs[0,1].legend(loc="upper left")

axs[1,0].plot(time,omega_des,label='Setpoint')
axs[1,0].plot(time,kp_12,label='KP=12')
axs[1,0].set_xlim([0,25])
axs[1,0].set_ylim([-10,30])
axs[1,0].set_xlabel('Time [s]')
axs[1,0].set_ylabel('Motor Speed [rad/s]')
# axs[1,0].set_title('KP=12')
axs[1,0].legend(loc="upper left")

axs[1,1].plot(time,omega_des,label='Setpoint')
axs[1,1].plot(time,kp_16,label='KP=16')
axs[1,1].set_xlim([0,25])
axs[1,1].set_ylim([-10,30])
axs[1,1].set_xlabel('Time [s]')
#axs[1,1].set_ylabel('Motor Speed [rad/s]')
# axs[1,1].set_title('KP=16')
axs[1,1].legend(loc="upper left")

plt.show()

##################################################################
#                            phase 2                             #
##################################################################

phase2_df = pd.read_csv('24-11-11_PI-tune_PHASE-2_I-varied_P08.csv',
                        header=1,
                        sep=',')

phase2_array = phase2_df.values
time = phase2_array[:,0]
omega_des = phase2_array[:,1]

# single initial plot
#####################

plt.plot(time, omega_des, label='Setpoint')
plt.plot(time, phase2_df['KI=0.01'], label='KI=0.01')

plt.xlim([0,25])
plt.ylim([0,30])
plt.xlabel('Time [s]')
plt.ylabel('Motor Speed [rad/s]')
plt.title('Phase 2: Integral Tuning, KP=8')
plt.legend()
plt.show()

# overlay all
######################
gains = phase2_df.columns[2:-1].tolist()
plt.plot(time, omega_des, label='Setpoint')

for gain in gains:
    plt.plot(time,phase2_df[gain],label=gain)

plt.xlim([0,25])
plt.ylim([-10,30])
plt.xlabel('Time [s]')
plt.ylabel('Motor Speed [rad/s]')
plt.title('Phase 2: Integral Tuning, KP=8')
plt.legend()
plt.show()

# overlay limited
######################
gains = phase2_df.columns[2:-1].tolist()
plt.plot(time, omega_des, label='Setpoint')

gains_subset = [gains[0],gains[1],gains[2],gains[4],gains[7]]

for gain in gains_subset:
    plt.plot(time,phase2_df[gain],label=gain)

plt.xlim([0,25])
plt.ylim([-10,30])
plt.xlabel('Time [s]')
plt.ylabel('Motor Speed [rad/s]')
plt.title('Phase 2: Integral Tuning, KP=8')
plt.legend()
plt.show()

# overlay limited zoomed
######################
gains = phase2_df.columns[2:-1].tolist()
plt.plot(time, omega_des, label='Setpoint')

gains_subset = [gains[0],gains[1],gains[2],gains[4],gains[7]]

for gain in gains_subset:
    plt.plot(time,phase2_df[gain],label=gain)

plt.xlim([10,25])
plt.ylim([10,30])
plt.xlabel('Time [s]')
plt.ylabel('Motor Speed [rad/s]')
plt.title('Phase 2: Integral Tuning, KP=8')
plt.legend()
plt.show()

# subplot
######################

fig, axs = plt.subplots(2,2,figsize=(9,7))
fig.suptitle("Phase 2: Integral Tuning, KP=8")

axs[0,0].plot(time,omega_des,label='Setpoint')
ki = 'KI=0.01'
axs[0,0].plot(time,phase2_df[ki],label=ki)
axs[0,0].set_xlim([0,25])
axs[0,0].set_ylim([-10,30])
#axs[0,0].set_xlabel('Time [s]')
axs[0,0].set_ylabel('Motor Speed [rad/s]')
axs[0,0].set_title(ki)

axs[0,1].plot(time,omega_des,label='Setpoint')
ki = 'KI=0.025'
axs[0,1].plot(time,phase2_df[ki],label=ki)
axs[0,1].set_xlim([0,25])
axs[0,1].set_ylim([-10,30])
#axs[0,1].set_xlabel('Time [s]')
#axs[0,1].set_ylabel('Motor Speed [rad/s]')
axs[0,1].set_title(ki)

axs[1,0].plot(time,omega_des,label='Setpoint')
ki = 'KI=0.05'
axs[1,0].plot(time,phase2_df[ki],label=ki)
axs[1,0].set_xlim([0,25])
axs[1,0].set_ylim([-10,30])
axs[1,0].set_xlabel('Time [s]')
axs[1,0].set_ylabel('Motor Speed [rad/s]')
axs[1,0].set_title(ki)

axs[1,1].plot(time,omega_des,label='Setpoint')
ki = 'KI=0.06'
axs[1,1].plot(time,phase2_df[ki],label=ki)
axs[1,1].set_xlim([0,25])
axs[1,1].set_ylim([-10,30])
axs[1,1].set_xlabel('Time [s]')
#axs[1,1].set_ylabel('Motor Speed [rad/s]')
axs[1,1].set_title(ki)

plt.show()

# subplot
######################

fig, axs = plt.subplots(2,2,figsize=(9,7))
fig.suptitle("Phase 2: Integral Tuning, KP=8")

axs[0,0].plot(time,omega_des,label='Setpoint')
ki = 'KI=0.06'
axs[0,0].plot(time,phase2_df[ki],label=ki)
axs[0,0].set_xlim([0,25])
axs[0,0].set_ylim([-10,30])
#axs[0,0].set_xlabel('Time [s]')
axs[0,0].set_ylabel('Motor Speed [rad/s]')
axs[0,0].set_title(ki)

axs[0,1].plot(time,omega_des,label='Setpoint')
ki = 'KI=0.07'
axs[0,1].plot(time,phase2_df[ki],label=ki)
axs[0,1].set_xlim([0,25])
axs[0,1].set_ylim([-10,30])
#axs[0,1].set_xlabel('Time [s]')
#axs[0,1].set_ylabel('Motor Speed [rad/s]')
axs[0,1].set_title(ki)

axs[1,0].plot(time,omega_des,label='Setpoint')
ki = 'KI=0.08'
axs[1,0].plot(time,phase2_df[ki],label=ki)
axs[1,0].set_xlim([0,25])
axs[1,0].set_ylim([-10,30])
axs[1,0].set_xlabel('Time [s]')
axs[1,0].set_ylabel('Motor Speed [rad/s]')
axs[1,0].set_title(ki)

axs[1,1].plot(time,omega_des,label='Setpoint')
ki = 'KI=0.09'
axs[1,1].plot(time,phase2_df[ki],label=ki)
axs[1,1].set_xlim([0,25])
axs[1,1].set_ylim([-10,30])
axs[1,1].set_xlabel('Time [s]')
#axs[1,1].set_ylabel('Motor Speed [rad/s]')
axs[1,1].set_title(ki)

plt.show()

# subplot
######################

fig, axs = plt.subplots(2,2,figsize=(9,7))
fig.suptitle("Phase 2: Integral Tuning, KP=8")

axs[0,0].plot(time,omega_des,label='Setpoint')
ki = 'KI=0.07'
axs[0,0].plot(time,phase2_df[ki],label=ki)
axs[0,0].set_xlim([0,25])
axs[0,0].set_ylim([-10,30])
#axs[0,0].set_xlabel('Time [s]')
axs[0,0].set_ylabel('Motor Speed [rad/s]')
# axs[0,0].set_title(ki)
axs[0,0].legend(loc="upper left")

axs[0,1].plot(time,omega_des,label='Setpoint')
ki = 'KI=0.08'
axs[0,1].plot(time,phase2_df[ki],label=ki)
axs[0,1].set_xlim([0,25])
axs[0,1].set_ylim([-10,30])
#axs[0,1].set_xlabel('Time [s]')
#axs[0,1].set_ylabel('Motor Speed [rad/s]')
# axs[0,1].set_title(ki)
axs[0,1].legend(loc="upper left")

axs[1,0].plot(time,omega_des,label='Setpoint')
ki = 'KI=0.09'
axs[1,0].plot(time,phase2_df[ki],label=ki)
axs[1,0].set_xlim([0,25])
axs[1,0].set_ylim([-10,30])
axs[1,0].set_xlabel('Time [s]')
axs[1,0].set_ylabel('Motor Speed [rad/s]')
# axs[1,0].set_title(ki)
axs[1,0].legend(loc="upper left")

axs[1,1].plot(time,omega_des,label='Setpoint')
ki = 'KI=0.1'
axs[1,1].plot(time,phase2_df[ki],label=ki)
axs[1,1].set_xlim([0,25])
axs[1,1].set_ylim([-10,30])
axs[1,1].set_xlabel('Time [s]')
#axs[1,1].set_ylabel('Motor Speed [rad/s]')
# axs[1,1].set_title(ki)
axs[1,1].legend(loc="upper left")

plt.show()


##################################################################
#                            phase 3                             #
##################################################################

phase3_df = pd.read_csv('24-11-11_PI-tune_PHASE-3_I0x08_P-varied.csv',
                        header=1,
                        sep=',')

phase3_array = phase3_df.values
time = phase3_array[:,0]
omega_des = phase3_array[:,1]

# overlay all
######################
gains = phase3_df.columns[2:].tolist()
plt.plot(time, omega_des, label='Setpoint')

for gain in gains:
    plt.plot(time,phase3_df[gain],label=gain)

plt.xlim([0,25])
plt.ylim([-15,30])
plt.xlabel('Time [s]')
plt.ylabel('Motor Speed [rad/s]')
plt.title('Phase 3: Integral Detuning, KI=0.08')
plt.legend()
plt.show()

# overlay powers of 2
######################
plt.plot(time, omega_des, label='Setpoint')

gains_subset = [gains[0],gains[1],gains[2],gains[4],gains[6]]

for gain in gains_subset:
    plt.plot(time,phase3_df[gain],label=gain)

plt.xlim([0,25])
plt.ylim([-15,30])
plt.xlabel('Time [s]')
plt.ylabel('Motor Speed [rad/s]')
plt.title('Phase 3: Integral Detuning, KI=0.08')
plt.legend()
plt.show()

# subplot
######################

fig, axs = plt.subplots(1,2,figsize=(9,4))
fig.suptitle("Phase 3: Integral Detuning, KI=0.08")

axs[0].plot(time,omega_des,label='Setpoint')
for gain in gains_subset:
    axs[0].plot(time,phase3_df[gain],label=gain)
axs[0].set_xlim([0,25])
axs[0].set_ylim([-15,30])
axs[0].set_xlabel('Time [s]')
axs[0].set_ylabel('Motor Speed [rad/s]')
axs[0].legend(loc='upper left')
# axs[0,0].set_title(KP)

axs[1].plot(time,omega_des,label='Setpoint')
for gain in gains_subset:
    axs[1].plot(time,phase3_df[gain],label=gain)
axs[1].set_xlim([19.5,20.5])
axs[1].set_xticks([19.5,19.75,20,20.25,20.5])
axs[1].set_ylim([-15,30])
axs[1].set_xlabel('Time [s]')
axs[1].legend(loc='upper left')
# axs[1].set_ylabel('Motor Speed [rad/s]')

plt.show()

# subplot
######################

fig, axs = plt.subplots(2,2,figsize=(9,7))
fig.suptitle("Phase 3: Integral Detuning, KI=0.08")

axs[0,0].plot(time,omega_des,label='Setpoint')
KP = 'KP=0.5'
axs[0,0].plot(time,phase3_df[KP],label=KP)
axs[0,0].set_xlim([0,25])
axs[0,0].set_ylim([-15,30])
#axs[0,0].set_xlabel('Time [s]')
axs[0,0].set_ylabel('Motor Speed [rad/s]')
axs[0,0].set_title(KP)

axs[0,1].plot(time,omega_des,label='Setpoint')
KP = 'KP=1'
axs[0,1].plot(time,phase3_df[KP],label=KP)
axs[0,1].set_xlim([0,25])
axs[0,1].set_ylim([-15,30])
#axs[0,1].set_xlabel('Time [s]')
#axs[0,1].set_ylabel('Motor Speed [rad/s]')
axs[0,1].set_title(KP)

axs[1,0].plot(time,omega_des,label='Setpoint')
KP = 'KP=2'
axs[1,0].plot(time,phase3_df[KP],label=KP)
axs[1,0].set_xlim([0,25])
axs[1,0].set_ylim([-15,30])
axs[1,0].set_xlabel('Time [s]')
axs[1,0].set_ylabel('Motor Speed [rad/s]')
axs[1,0].set_title(KP)

axs[1,1].plot(time,omega_des,label='Setpoint')
KP = 'KP=3'
axs[1,1].plot(time,phase3_df[KP],label=KP)
axs[1,1].set_xlim([0,25])
axs[1,1].set_ylim([-15,30])
axs[1,1].set_xlabel('Time [s]')
#axs[1,1].set_ylabel('Motor Speed [rad/s]')
axs[1,1].set_title(KP)

plt.show()

# overlay initlal and final
######################
plt.plot(time, omega_des, label='Setpoint')

gains_subset = [gains[6],gains[1]]

for gain in gains_subset:
    plt.plot(time,phase3_df[gain],label=gain)

plt.xlim([0,25])
plt.ylim([-15,30])
plt.xlabel('Time [s]')
plt.ylabel('Motor Speed [rad/s]')
plt.title('Phase 3: Integral Detuning, KI=0.08')
plt.legend()
plt.show()
