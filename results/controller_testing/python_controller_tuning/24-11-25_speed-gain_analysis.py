import pandas as pd
import matplotlib.pyplot as plt

def extract_python_controller_data(path, header_loc, column_of_interest):
    df = pd.read_csv(path, header=header_loc, sep=',')
    array = df.values

    time = array[:,0]
    data = array[:,column_of_interest]

    return [time, data]

gain_strings = ['0010', '0015', '0020', '0025', '0030']

# short step
######################################################
short_dic = {}
for gain in gain_strings:
    path = 'speed_short_f500/24-11-25_speed-gain-test_short_0x' + gain + '_00.csv'
    short_dic[gain]=extract_python_controller_data(path, 21, 1)

    my_label = 'Ks=0.' + gain
    plt.plot(short_dic[gain][0],short_dic[gain][1],label=my_label)

plt.xlim([0.8,10])
plt.ylim([-100,300])
plt.xlabel('Time [s]')
plt.ylabel('Distance [mm]')
plt.title('Speed gain, short step 250mm')
plt.legend()
plt.show()

# long step
######################################################
long_dic = {}
for gain in gain_strings:
    path = 'speed_long_f500/24-11-25_speed-gain-test_long_0x' + gain + '_00.csv'
    long_dic[gain]=extract_python_controller_data(path, 21, 1)

    my_label = 'Ks=0.' + gain
    plt.plot(long_dic[gain][0],long_dic[gain][1],label=my_label)

plt.xlim([0.8,10])
plt.ylim([-250,1250])
plt.xlabel('Time [s]')
plt.ylabel('Distance [mm]')
plt.title('Speed gain, long step 1000mm')
plt.legend()
plt.show()

# both steps
######################################################
for this_dic in [short_dic, long_dic]:
    for key in this_dic:
        my_label = 'Ks=0.' + key
        plt.plot(this_dic[key][0],this_dic[key][1],label=my_label)

plt.xlim([0.8,10])
plt.ylim([-250,1250])
plt.xlabel('Time [s]')
plt.ylabel('Distance [mm]')
plt.title('Speed gain, short and long steps')
plt.show()

# side by side
#################################################

this_dic = short_dic
plt.figure(figsize=(11,4.8))
plt.subplot(1,2,1)
for key in this_dic:
    my_label = 'Ks=0.' + key
    plt.plot(this_dic[key][0],this_dic[key][1],label=my_label)
plt.xlim([0.8,10])
plt.ylim([-250,1250])
plt.xlabel('Time [s]')
plt.ylabel('Distance [mm]')
plt.title('Short Step (250 mm)')
plt.legend()

this_dic = long_dic
plt.subplot(1,2,2)
for key in this_dic:
    my_label = 'Ks=0.' + key
    plt.plot(this_dic[key][0],this_dic[key][1],label=my_label)
plt.xlim([0.8,10])
plt.ylim([-250,1250])
plt.xlabel('Time [s]')
#plt.ylabel('Distance [mm]')
plt.title('Long Step (1000 mm)')
plt.legend()

plt.suptitle('Speed Gain')
plt.show()
