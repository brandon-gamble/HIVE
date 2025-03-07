import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

# run with idle 3.6

def extract_python_data(path, header_loc):#, column_of_interest):
    df = pd.read_csv(path, header=header_loc, sep=',')
    return df

path = 'theta-pov-81d-GOOD/known-obj-dist-est-calibration-verification.csv'

df_raw = extract_python_data(path, 7)

dist =        list(df_raw['True Dist [mm]'].values)
rs_error =    list(df_raw[' RS Error [%]'].values)
trig_error =  list(df_raw[' Pixel Theo Error [%]'].values)
calib_error = list(df_raw[' Pixel Calib Error [%]'].values)
k = len(dist)

################################################################################
# only rs
################################################################################
method=['RealSense']*k
rs_dic = {'Distance [mm]':dist,
    'Error [%]':rs_error,
    'Method':method}
df = pd.DataFrame(rs_dic)

plt.figure(figsize=(7,5))
sns.set_style('whitegrid')
sns.boxplot(x='Distance [mm]', y='Error [%]', hue='Method', data=df,
            showfliers=False)
plt.title('Error in Distance Approximation')

plt.show()
################################################################################
# only trig
################################################################################
method=['Pure Trigonometry']*k
trig_dic = {'Distance [mm]':dist,
    'Error [%]':trig_error,
    'Method':method}
df = pd.DataFrame(trig_dic)

plt.figure(figsize=(7,5))
sns.set_style('whitegrid')
sns.boxplot(x='Distance [mm]', y='Error [%]', hue='Method', data=df,
            showfliers=False)
plt.title('Error in Distance Approximation of Object of Known Size')

plt.show()
################################################################################
# only calib
################################################################################
method=['Calibrated Trigonometry']*k
calib_dic = {'Distance [mm]':dist,
    'Error [%]':calib_error,
    'Method':method}
df = pd.DataFrame(calib_dic)

plt.figure(figsize=(7,5))
sns.set_style('whitegrid')
sns.boxplot(x='Distance [mm]', y='Error [%]', hue='Method', data=df,
            showfliers=False)
plt.title('Error in Distance Approximation of Object of Known Size')

plt.show()
################################################################################
# all together
################################################################################
dist_list = dist*3
error_list = rs_error + trig_error + calib_error
method_list = ['RealSense']*k + ['Pure Trigonometry']*k + ['Calibrated Trigonometry']*k

dic = {'Distance [mm]':dist_list,
    'Error [%]':error_list,
    'Method':method_list}

df = pd.DataFrame(dic)

plt.figure(figsize=(7,5))
sns.set_style('whitegrid')
sns.boxplot(x='Distance [mm]', y='Error [%]', hue='Method', data=df,
            showfliers=False)
plt.title('Error in Distance Approximation')

plt.show()

################################################################################

# mask_values = np.arange(200,1600,100)
#
# error_set = []
# for mask_val in mask_values:
#     mask = data[0] == mask_val
#     error = data[1][mask]
#     error_set.append(error)
#
# plt.boxplot(error_set)
# plt.show()
# plt.xticks(list(np.arange(1,15,1)),[str(element) for element in np.arange(200,1600,100)])

# columns
# 0, True Dist [mm],
# 1, RS Meas Dist [mm],
# 2, RS Error [%],
# 3, Pixel Approx Dist Theoretical [mm],
# 4, Pixel Theo Error [%],
# 5, Pixel Approx Dist Calib [mm],
# 6, Pixel Calib Error [%]

# true_dist = extract_python_controller_data(path, 7, 0)
# rs_dist = extract_python_controller_data(path, 7, 1)
# rs_error = extract_python_controller_data(path, 7, 2)
# trig_dist = extract_python_controller_data(path, 7, 3)
# trig_error = extract_python_controller_data(path, 7, 4)
# calib_dist = extract_python_controller_data(path, 7, 5)
# calib_error = extract_python_controller_data(path, 7, 6)
#
# plt.boxplot(rs_error)
# plt.show()
