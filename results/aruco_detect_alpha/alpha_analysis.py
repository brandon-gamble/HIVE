import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv('determine_alpha.csv',header=1,sep=',')


plt.plot(df[' Angle [deg]'],df[' Size [px]'],'.')
plt.xlabel('Angle [deg]')
plt.ylabel('Marker Size [px]')
plt.xlim([0,90])
plt.title('30-50mm Marker Detection')
plt.show()

plt.plot(df[' Angle [deg]'],df[' Alpha [px/bit]'],'.')
plt.xlabel('Angle [deg]')
plt.ylabel('Gamma [px/bit]')
plt.xlim([-1,46])
plt.ylim([1.2,2.2])
plt.title('30-50mm Marker Detection: Gamma Envelope')
plt.show()
