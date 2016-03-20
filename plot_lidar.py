import numpy as np
import matplotlib.pyplot as plt
import math
import pdb
import sys

def plot_file(fname):
    """slurp lidar data from a txt file and plot it with pyplot"""
    data = np.loadtxt(fname, delimiter=",")

    # toss out the error data
    error_free_data = np.array(filter(lambda x: x[1] < 777, data))

    # extract the theta and radius and convert to radians
    theta,radius = zip(*error_free_data)
    theta = map(math.radians, theta)

    # Now do the plot
    fig = pylab.figure(figsize=(9,9))
    ax = fig.add_subplot(111, projection='polar')
    ax.set_rmax(240)
    ax.grid(True)
    ax.cla()
    ax.plot(theta, radius, 'x', color='r', linewidth=3)
    plt.show()

for arg in sys.argv[1:]:
    plot_file(arg)
    
