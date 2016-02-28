import pylab
import math
import logging

class LidarLogger ():
    def __init__ (self, logger):
        self.logger = logger

    def log_data(self, polar_data):
        """Export lidar data to a file"""
        self.logger.info("BEGIN Full rotation of sanitized lidar data")
        for theta,radius in polar_data:
            self.logger.info("{:d}, {:.1f}\n".format(theta, radius))
        self.logger.info("END Full rotation of sanitized lidar data")

class LidarViewer (object):
    """Provide plotting, exporting, and importing services for a rotation"""
    begin_marker = 888
    end_marker = 999
    """Draw non-blocking pyplot of the polar_data for a rotation"""
    @staticmethod
    def round_degrees(theta, radius):
        return int(round(theta)), radius
    
    def __init__ (self):
        self.fig = pylab.figure(figsize=(9,9))
        self.ax = self.fig.add_subplot(111, projection='polar')
        self.ax.set_rmax(240)
        self.ax.grid(True)

    def plot (self, rotation):
        self.plot_polar(rotation)
        
    def plot_polar (self, polar_data):
        theta, radius = zip(*polar_data)
        theta = map(math.radians, theta)
        robot_r = [ -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5 ]
        robot_theta = [0]*11
        self.ax.cla()
        self.ax.plot(theta, radius, 'x', color='r', linewidth=3)
        self.ax.plot(robot_theta, robot_r, 'x', color='b', linewidth=3)
        pylab.show(block=False)
        
    def plot_cartesian (self, cartesian_data):
        """
        Do a simple plot of cartesian data.   Not generally useful,
        except for plotting the cartesian version of the field model.
        """
        self.fig = pylab.figure(figsize=(9,9))
        self.axes = self.fig.add_subplot(111,aspect='equal')
        self.axes.set_autoscale_on(False)
        self.axes.set_xlim([-FieldModel.field_width,FieldModel.field_width])
        self.axes.set_ylim([-FieldModel.field_width,FieldModel.field_width])

        x, y  = zip(*cartesian_data)

        self.axes.plot(x, y, 'x')
        pylab.show(block=False)

