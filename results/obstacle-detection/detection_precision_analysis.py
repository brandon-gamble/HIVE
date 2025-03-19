import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np


def extract_python_data(path, header_loc,id):#, column_of_interest):
    df = pd.read_csv(path, header=header_loc, sep=',')
    df.insert(0,'Test_ID',[id]*100)
    return df

test_ids = ['00','01','02','03','04','05','06','07','08','09','10','11','12','13','14']

df = pd.DataFrame()
for id in test_ids:
    path = '25-03-19_detection_precision_test_' + id + ".csv"
    df = pd.concat([df, extract_python_data(path, 10, id)])

# compensate for error in algorithm
df['F_E [%]'] += 20

################################################################################
# plot separately to inspect each trial
################################################################################
fig, axes = plt.subplots(1,5)
fig.set_size_inches(20,4)
#############################
# plot yaw error left/right #
#############################
ylim = 30
sns.boxplot(x='Test_ID',y='yaw_L_E [%]',data=df,showfliers=False, ax=axes[0])
axes[0].set_ylim([-ylim,ylim])
# axes[0].set_title('Title')
sns.boxplot(x='Test_ID',y='yaw_R_E [%]',data=df,showfliers=False, ax=axes[1])
axes[1].set_ylim([-ylim,ylim])
##############################
# plot dist error left/right #
##############################
ylim = 7.5
sns.boxplot(x='Test_ID',y='D_L_E [%]',data=df,showfliers=False, ax=axes[2])
axes[2].set_ylim([-ylim,ylim])
sns.boxplot(x='Test_ID',y='D_R_E [%]',data=df,showfliers=False, ax=axes[3])
axes[3].set_ylim([-ylim,ylim])
#############################
# plot face length error    #
#############################
ylim = 25
sns.boxplot(x='Test_ID',y='F_E [%]',data=df,showfliers=False, ax=axes[4])
axes[4].set_ylim([-ylim,ylim])

# subplot titles
axes[0].set_title('Left Yaw')
axes[1].set_title('Right Yaw')
axes[2].set_title('Left Distance')
axes[3].set_title('Right Distance')
axes[4].set_title('Face Length')
plt.suptitle('Error in Obstacle Detection Parameters (per Trial)')

# y axis labels
axes[0].set_ylabel('Error [%]')
axes[1].set_ylabel('Error [%]')
axes[2].set_ylabel('Error [%]')
axes[3].set_ylabel('Error [%]')
axes[4].set_ylabel('Error [%]')

# y axis labels
# axes[0].set_ylabel('')
axes[1].set_ylabel('')
axes[2].set_ylabel('')
axes[3].set_ylabel('')
axes[4].set_ylabel('')

plt.show()

################################################################################
# make new dataframes sorted by parameter
# whisker plot by parameter
################################################################################

#####################
# put yaws together #
#####################
yaw_l_col = df['yaw_L_E [%]']
yaw_r_col = df['yaw_R_E [%]']
param = ['Yaw']*len(yaw_l_col)

yaw_l_df = pd.DataFrame({'Parameter':param, 'Error [%]':yaw_l_col})
yaw_r_df = pd.DataFrame({'Parameter':param, 'Error [%]':yaw_r_col})
yaw_df = pd.concat([yaw_l_df, yaw_r_df])

######################
# put dists together #
######################
dist_l_col = df['D_L_E [%]']
dist_r_col = df['D_R_E [%]']
param = ['Distance']*len(dist_l_col)

dist_l_df = pd.DataFrame({'Parameter':param, 'Error [%]':dist_l_col})
dist_r_df = pd.DataFrame({'Parameter':param, 'Error [%]':dist_r_col})
dist_df = pd.concat([dist_l_df, dist_r_df])

####################
# make face length #
####################
f_col = df['F_E [%]']
param = ['Face Length']*len(f_col)

f_df = pd.DataFrame({'Parameter':param, 'Error [%]':f_col})

#########################
# put all df's together #
#########################
parsed_df = pd.concat([yaw_df, dist_df, f_df])

#####################
# make whisker plot #
#####################
sns.boxplot(x='Parameter',y='Error [%]',data=parsed_df,showfliers=False)
plt.title('Error in Obstacle Detection Parameters')
plt.show()

################################################################################
################################################################################
