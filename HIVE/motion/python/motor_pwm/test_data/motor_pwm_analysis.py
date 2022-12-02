import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['backend'] = 'TkAgg'
plt.rcParams["figure.figsize"] = [16, 3.50]
plt.rcParams["figure.autolayout"] = True

# Function to print mouse click event coordinates
# improper use of global but if it ain't broke don't fix it
global times
times = []
def freq_onclick(event):
    precision = 5 # sig figs to print
    times.append(event.xdata) # append x coord time to list

    if len(times) > 1:
        # calculate period between current and last time clicked
        period = times[-1] - times[-2]

        # build and print period and freq strings
        period_str = 'PERIOD: ' + str(period*1000)[0:precision+1] + ' [ms]'
        freq_str = 'FREQ: ' + str(1/period)[0:precision+1] + ' [Hz]'
        out_str = period_str + ' ' + freq_str
        print(out_str)

    return times

def time_onclick(event):
    # get time at x coord of mouse click
    global this_time
    this_time = event.xdata
    plt.close()

    return


def analyze_frequency(filename):
    # read in data
    df = pd.read_csv(filename, skiprows=5)
    df.columns=['id','time','volt1','volt2','na']

    # Create a figure and a set of subplots
    fig, ax = plt.subplots()

    # Plot a line in the range of 10
    ax.plot(df.time,df.volt1)

    # Bind the button_press_event with the freq_onclick() method
    fig.canvas.mpl_connect('button_press_event', freq_onclick)

    # Display the plot
    plt.show()

    return df

def analyze_time(filename):
    # read in data
    df = pd.read_csv(filename, skiprows=5)
    df.columns=['id','time','volt1','volt2','na']

    # Create a figure and a set of subplots
    fig, ax = plt.subplots()

    # Plot a line in the range of 10
    ax.plot(df.time,df.volt1)

    # Bind the button_press_event with the time_onclick() method
    fig.canvas.mpl_connect('button_press_event', time_onclick)

    # Display the plot
    plt.show()

    return df

if __name__ == "__main__":
    date = '2022-11-30'
    low = 0
    high = 22

    options = ['frequency','time']
    lookfor = options[1]

    for k in range(low,high+1):
        # build filename
        filename= date + '_' + str(k).zfill(2) + '.csv'
        print(filename)

        if lookfor == 'frequency':
            # open plot to analyze
            df = analyze_frequency(filename)

        if lookfor == 'time':
            df = analyze_time(filename)
            print(this_time)
            print(df[df.time>this_time])
