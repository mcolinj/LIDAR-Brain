"""
This plays back display of lidar snapshot logs.
Arguments are the name of the data folder, the start file index number, and the number of seconds
per frame of data.   (it is a sleep time after each plot)
"""

from lidar_viewer import LidarViewer
import numpy as np
import sys
import time
import pdb


#
#   Open up the serial port, get lidar data and write it to a file
#   every few seconds.
#
if __name__ == '__main__':

        if len(sys.argv) > 1:
                image_directory = sys.argv[1]

        if len(sys.argv) > 2:
                start_index = int(sys.argv[2])
        else:
                first_index = 1000

        if len(sys.argv) > 3:
                seconds_per_plot = int(sys.argv[3])

        lidar_viewer = LidarViewer()

        file_index = start_index

        while 1:
                fname = "{:s}/lidar_snapshot_{:d}.dat".format(image_directory, file_index)

                lidar_data = []
                print("Loading file: {:s}\n".format(fname))
                lidar_data = np.loadtxt(fname, delimiter=",")

                # toss out the error data
                error_free_data = []
                if len(lidar_data) > 0:
                        error_free_data = np.array(filter(lambda x: x[1] < 777, lidar_data))

                if len(error_free_data) > 0:
                        print(error_free_data)
                        pdb.set_trace()
                        lidar_viewer.plot_polar(error_free_data)

                file_index = file_index + 1
