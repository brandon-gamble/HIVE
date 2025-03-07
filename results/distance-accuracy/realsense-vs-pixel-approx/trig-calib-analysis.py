import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

def extract_python_data(path, header_loc, column_of_interest):
    df = pd.read_csv(path, header=header_loc, sep=',')
    array = df.values

    time = array[:,0]
    data = array[:,column_of_interest]

    return [time, data]

def power_law(x,a,b):
    return a * np.power(x,b)

path = 'theta-pov-81d-GOOD/pixel-corelation.csv'

data = extract_python_data(path, 3, 1,)
x_data = data[1]
y_data = data[0]

# Fit the power law function to the data
popt, pcov = curve_fit(power_law, x_data, y_data)

# Extract the fitted parameters
a_fit, b_fit = popt

# find r2 value
residuals = y_data - power_law(x_data, *popt)
ss_res = np.sum(residuals**2)
ss_tot = np.sum((y_data-np.mean(y_data))**2)
r_squared = 1 - (ss_res/ss_tot)

print("R**2: " + str(r_squared))

# Generate points for the fitted curve
x_fit = np.linspace(min(x_data), max(x_data), 100)
y_fit = power_law(x_fit, a_fit, b_fit)

plt.plot(x_data,y_data,'o', label='Experimental Data')
plt.plot(x_fit,y_fit, label=f'Fit: d = {a_fit:.0f} * p^{b_fit:.3f} \n R2 = {r_squared:.4f}')
plt.xlabel('Pixel Size [px]')
plt.ylabel('Distance [mm]')
plt.title('Distance vs Pixel Size of 45mm Marker')
plt.legend()
plt.show()
