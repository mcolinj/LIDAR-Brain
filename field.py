import numpy as np
import itertools
import math
from analyzer import *
from lidar_viewer import LidarLogger, LidarViewer
from field_model import FakeRotation, FieldModel, Robot
import pdb

def update_all_results (lidar_viewer, field_model, robot, factor):
    """generate synthetic lidar data based on model and analyze/plot"""
    rotation = FakeRotation(field_model, robot)

    # do simple sweep analysis and report
    sweep_heading, sweep_range = Analyzer.range_at_heading(rotation.polar_data(), (-5, 6))
    print("Closest point in (-5,6) sweep is ({:d},{:.2f})".format(sweep_heading, sweep_range))

    # do some analysis and gather up the results so they can be plotted
    # run a 4-point r-squared and collect those less than threshold value
    cart_data = rotation.cartesian_data()
    r2_markers = []
    building_vector_start = None
    building_vector_end = None
    max_mag = 0
    for i in range(0, len(cart_data)-factor):
        slice = cart_data[i:i+factor]
        r2 = r_squared(slice)
        # start or continue
        if r2 > .9:
            if building_vector_start is not None:
                building_vector_end = slice[0]
            else:
                building_vector_start = slice[0]
                building_vector_end = slice[0]
        # continue
        if r2 > .5:
            if building_vector_start is not None:
                building_vector_end = slice[0]
        # found end of a vector.    Display it.
        if building_vector_start is not None and r2 <= 0.5:
            x1, y1 = building_vector_start
            x2, y2 = building_vector_end
            new_mag = math.sqrt((x2-x1)**2+(y2-y1)**2)
            print("mag {:.1f} vector going from ({:.1f},{:.1f}) to ({:.1f},{:.1f})\n".format(new_mag, x1, y1, x2, y2))
            if new_mag > max_mag:
                max_mag = new_mag
                max_start = building_vector_start
                max_end = building_vector_end
            building_vector_start = None
            building_vector_end = None
        
        if r2 < 0.8:
            x,y = cart_data[i+factor]
            print("r2 = {:.2f} at {:.2f},{:2f}\n".format(r2,x,y))
            r2_markers.append((x,y))

    #  plot the polar data for reference
    lidar_viewer.plot_polar(rotation.polar_data())

    #  plot the rectangular data
    lidar_viewer.plot_cartesian(cart_data)

    #
    # display the average distance of data points
    # might be used to dynamically choose the r2 thresholds
    # or dynamically select the r-factor for analysis?
    avg_dist = avg_distance(cart_data)
    print("Average distance: {:.1f}\n".format(avg_dist))

    # plot any calculated markers in their specified color
    lidar_viewer.plot_markers(r2_markers, 'g')

    wall_markers = [max_start, max_end]
    lidar_viewer.plot_markers(wall_markers, 'b')


if __name__ == '__main__':

    # put robot at the origin
    robot = Robot((0,0), 0)
    field_model = FieldModel()
    lidar_viewer = LidarViewer()
    factor = 4
        
    while True:
        text = raw_input("Enter command (mOVE,tURN,sHOW,iNFO,0(origin),hELP or qQUIT): ")
        cmd  = text.split(" ")[0]
        
        #If we see a quit command, well, quit
        if cmd == "q":
            break
            
        #Reset robot to the origin with 0 turn
        if cmd == "0":
            robot = Robot((0,0), 0)
        elif cmd == "i":
            x,y = robot.position
            print("Robot is at ({:f}, {:f} turned {:d} degrees".format(x, y, r.heading))
        elif cmd == "s":
            update_all_results(lidar_viewer, field_model, robot, factor)
        elif cmd == "r":
            try:
                cmd,cmd_arg = text.split(" ")
                factor = int(cmd_arg)
                update_all_results(lidar_viewer, field_model, robot, factor)

            except ValueError:
                print "Please specify distance in whole inches."            
        elif cmd == "m":
            try:
                cmd,cmd_arg = text.split(" ")
                distance = int(cmd_arg)
                robot.move(distance)
                update_all_results(lidar_viewer, field_model, robot, factor)

            except ValueError:
                print "Please specify distance in whole inches."
        elif cmd == "t":
            try:
                cmd,cmd_arg = text.split(" ")
                heading = int(cmd_arg)
                robot.turn(heading)
                update_all_results(lidar_viewer, field_model, robot, factor)
            except ValueError:
                print "Please specify turn in whole degrees."
        elif cmd == "w":
            lidar_viewer.write_to_file("Lidar.dat", rotation.polar_data())
        elif cmd == "h":
            print "Enter single letter command and optional argument."

