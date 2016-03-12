import math
import logging
import matplotlib.pyplot as plt
from field_model import FieldModel

class LidarLogger ():
    def __init__ (self, logger):
        self.logger = logger

    def log_data(self, polar_data):
        """Export lidar data to a file"""
        self.logger.info("BEGIN Full rotation of sanitized lidar data")
        for theta,radius in polar_data:
            self.logger.info("{:d}, {:.1f}\n".format(theta, radius))
        self.logger.info("END Full rotation of sanitized lidar data")

    @staticmethod
    def write_to_file(file_name, polar_data):
        with open(file_name, 'w') as f:
            for (theta, distance) in polar_data:
                f.write("{:.1f}, {:.1f}\n".format(theta, distance))


class LidarViewer (object):
    """Provide plotting, exporting, and importing services for a rotation"""
    begin_marker = 888
    end_marker = 999
    """Draw non-blocking pyplot of the polar_data for a rotation"""
    @staticmethod
    def round_degrees(theta, radius):
        return int(round(theta)), radius
    
    @staticmethod
    def cart_to_polar(x, y):
        """convert cartesian to polar data."""
        """Map (x,y) to (theta, radius).  Theta is in degrees."""
        return math.degrees(math.atan2(y, x)), math.sqrt(x**2+y**2)

    def __init__ (self):
        self.fig = plt.figure(figsize=(8,4))
        self.pax = self.fig.add_subplot(121, projection='polar')
        self.pax.set_rmax(240)
        self.pax.grid(True)

        self.cax = self.fig.add_subplot(122,aspect='equal')
        self.cax.set_autoscale_on(False)
        self.cax.set_xlim([-FieldModel.field_width/10,FieldModel.field_width/10])
        self.cax.set_ylim([-FieldModel.field_width/10,FieldModel.field_width/10])
        
    def plot (self, rotation):
        self.plot_polar(rotation)
        
    def plot_polar (self, polar_data):
        theta, radius = zip(*polar_data)
        theta = map(math.radians, theta)
        robot_r = [ -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5 ]
        robot_theta = [0]*11
        self.pax.cla()
        self.pax.plot(theta, radius, 'x', color='r', linewidth=3)
        self.pax.plot(robot_theta, robot_r, 'x', color='b', linewidth=3)
        plt.show(block=False)

    def plot_polar_markers (self, markers, marker_color):
        theta, radius = zip(*markers)
        theta = map(math.radians, theta)
        self.pax.plot(theta, radius, 'o', color=marker_color, linewidth=3)
        plt.show(block=False)

    def plot_markers (self, markers, marker_color):
        x, y = zip(*markers)
        self.cax.plot(x, y, 'o', color=marker_color, linewidth=3)
        plt.show(block=False)
        
    def plot_cartesian (self, cartesian_data):
        """
        Do a simple plot of cartesian data.   Not generally useful,
        except for plotting the cartesian version of the field model.
        """
        x, y  = zip(*cartesian_data)
        self.cax.cla()
        self.cax.plot(x, y, 'x', color='r')
        self.cax.set_xlim([-FieldModel.field_width/10,FieldModel.field_width/10])
        self.cax.set_ylim([-FieldModel.field_width/10,FieldModel.field_width/10])
        plt.show(block=False)

