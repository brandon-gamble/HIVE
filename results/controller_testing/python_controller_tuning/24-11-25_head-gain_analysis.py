import pandas as pd
import matplotlib.pyplot as plt

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

    my_label = 'Kh=' + gain
    plt.plot(small_dic[gain][0],small_dic[gain][1],label=my_label)

plt.xlim([0.8,5])
plt.ylim([-.25,0.4])
plt.xlabel('Time [s]')
plt.ylabel('Heading [rad]')
plt.title('Head gain, small step 10deg')
plt.legend()
plt.show()

# large angle
######################################################
large_dic = {}
for gain in gain_strings:
    path = 'head_large/24-11-25_head-gain-test_large_' + gain + 'x00_00.csv'
    large_dic[gain]=extract_python_controller_data(path, 21, 2)

    my_label = 'Kh=' + gain
    plt.plot(large_dic[gain][0],large_dic[gain][1],label=my_label)

plt.xlim([0.8,5])
plt.ylim([-.25,1])
plt.xlabel('Time [s]')
plt.ylabel('Heading [rad]')
plt.title('Head gain, large step 40deg')
plt.legend()
plt.show()

# both steps
######################################################
for this_dic in [small_dic, large_dic]:
    for key in this_dic:
        my_label = 'Ks=0.' + key
        plt.plot(this_dic[key][0],this_dic[key][1],label=my_label)

    for key in this_dic:
        my_label = 'Ks=0.' + key
        plt.plot(this_dic[key][0],this_dic[key][1],label=my_label)

plt.xlim([0.8,10])
plt.ylim([-250,1250])
plt.xlabel('Time [s]')
plt.ylabel('Heading [rad]')
plt.title('Head gain, small and large steps')
plt.show()
