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

    # try out the analyzer find_wall
    (heading, distance, orientation) = find_wall_midpoint(cart_data)
    print("Analyzer.find_wall() heading {:.1f}, range {:.1f}, orientation {:.1f})".format(heading, distance, orientation))

    #  plot the polar data for reference
    lidar_viewer.plot_polar(rotation.polar_data())

    #  plot the rectangular data
    lidar_viewer.plot_cartesian(cart_data)

    #  polar markers (heading in degrees)
    find_wall_markers = [ (-heading, distance) ]

    #
    # display the average distance of data points
    # might be used to dynamically choose the r2 thresholds
    # or dynamically select the r-factor for analysis?
    avg_dist = avg_distance(cart_data)
    print("Average distance: {:.1f}\n".format(avg_dist))

    # plot any calculated markers in their specified color
    #lidar_viewer.plot_markers(r2_markers, 'g')

    lidar_viewer.plot_polar_markers(find_wall_markers, 'r')

    #wall_markers = [max_start, max_end]
    #lidar_viewer.plot_markers(wall_markers, 'b')


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

