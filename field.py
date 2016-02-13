import numpy as np
import pylab
import matplotlib.pyplot as plt
import itertools
import math

MM_PER_INCH = 25.4

class LidarView (object):
    begin_marker = 999
    fig = pylab.figure(figsize=(9,9))
    ax = fig.add_subplot(111, projection='polar')
    ax.set_rmax(240)
    ax.grid(True)

    @staticmethod
    def round_values(r, theta):
        return int(round(r)), int(round(theta))
    
    """create a polar field that can be easily drawn or converted to some lidar data"""
    def __init__ (self, polar_data, robot):
        """create lidar view from a set of rotated/translated polar points in the model world"""
        # round the data to whole degrees, and whole inches (4 bits of lidar noise)
        polar_data = itertools.starmap(LidarView.round_values, polar_data)
        # sort the data by the angle so it is in scan order
        polar_data = sorted(polar_data, key=lambda x: x[1])
        # then group all of the points by degree measure
        polar_data = itertools.groupby(polar_data, lambda x: round(x[1]))
        # pick the closest one in each group because it shadows the others
        self.data = [min(interval, key=lambda x: x[0]) for _, interval in polar_data]
        self.origin = robot.position
        self.orientation = robot.heading

    def __getitem__ (self, index):
        return self.data[index]
    def plot (self):
        r, theta = zip(*self.data)
        theta = map(math.radians, theta)
        LidarView.ax.cla()
        LidarView.ax.plot(theta, r, 'x', color='r', linewidth=3)
        pylab.plot(block=False)
    def write (self, fname):
        with open(fname, "a") as text_file:
            text_file.write("{:d}, ({:f},{:f}), {:d}\n".format(LidarView.begin_marker,
                                                               self.origin[0],
                                                               self.origin[1],
                                                               int(round(self.orientation))))
            for r,theta in self.data:
                text_file.write("{:d}, {:d}\n".format(theta, r))
        
class Robot (object):
    def __init__ (self, origin, rotation):
        self.position = origin
        self.heading = rotation

    def turn(self, rotation):
        """turn the robot according to the specified amount (degrees)"""
        """left turn is negative and right turn is positive (that is graph backwards)"""
        self.heading = self.heading + rotation

    def move(self, amount):
        """move the robot in the direction that it is pointing for the specified number of inches"""
        # make a triangle with the
        self.position = self.position[0] + amount * math.sin(self.heading), self.position[1] + amount * math.cos(self.heading)
        
class Field (object):
    """Fake field cartesian coordinates this is just a rectangle with a bump in the middle of an end"""
    # half width of field in inches
    field_width = 180
    # working depth of the field (12 feet or so) in inches
    field_depth = 144
    # tower width along the back wall (8 feet, but here a 1/2 width(
    tower_width = 48
    # 1/2 width of the tower face in inches
    tower_face  = 18
    def __init__ (self):
        self.field_data = [(x, Field.field_depth)
                           for x in range(-Field.field_width, -Field.tower_width)]
        self.field_data.extend((x, Field.field_depth-Field.tower_width-x)
                               for x in range(-Field.tower_width, -Field.tower_face))
        self.field_data.extend((x, Field.field_depth-Field.tower_width+Field.tower_face)
                               for x in range(-Field.tower_face, Field.tower_face))
        self.field_data.extend((x, Field.field_depth-Field.tower_width+x)
                               for x in range(Field.tower_face, Field.tower_width))
        self.field_data.extend((x, Field.field_depth)
                               for x in range(Field.tower_width, Field.field_width))
        #self.fig = pylab.figure(figsize=(9,9))
        #self.axes = self.fig.add_subplot(111,aspect='equal')
        #self.axes.set_autoscale_on(False)
        #self.axes.set_xlim([-Field.field_width,Field.field_width])
        #self.axes.set_ylim([-Field.field_width,Field.field_width])

    def __getitem__ (self, index):
        return self.field_data[index]

    @staticmethod
    def cart_to_polar(x, y):
        """convert cartesian to polar data"""
        return math.sqrt(x**2+y**2), math.degrees(math.atan2(y, x))

    @staticmethod
    def translate(x, y, origin):
        """translate the data points for the robot origin"""
        return x - origin[0], y - origin[1]
    
    @staticmethod
    def rotate(r, theta, rot):
        """rotate the data points in space by degrees to reflect robot orientation"""
        return r, theta+rot

    def createLidarView (self, robot):
        """create a lidar view for a robot on the field"""
        new_origin = (Field.translate(x, y, robot.position) for x, y in iter(self))
        new_polar = (Field.cart_to_polar(x, y) for x, y in new_origin)
        return LidarView([Field.rotate(r, theta, robot.heading) for r, theta in new_polar], robot)

    def plot (self, robot):
        """show a picture of the field with the robot on it"""
        x, y  = zip(*self.field_data)

        pylab.cla()
        self.axes.cla()
        self.fig.clf()
        
        self.axes.plot(x, y)
        pylab.show(block=False)

if __name__ == '__main__':

        # put robot at the origin
        r = Robot((0,0), 0)
        f = Field()
        lv = None
        
        while True:
            
            text = raw_input("Enter command (mOVE,tURN,sHOW,iNFO,0(origin),hELP or qQUIT): ")

            cmd  = text.split(" ")[0]

            #If we see a quit command, well, quit
            if cmd == "q":
                break

            #Reset robot to the origin with 0 turn
            if cmd == "0":
                r = Robot((0,0), 0)
            elif cmd == "i":
                x,y = r.position
                print("Robot is at ({:f}, {:f} turned {:d} degrees".format(x, y, r.heading))
            elif cmd == "s":
                lv = f.createLidarView(r)
                lv.plot()
            elif cmd == "m":
                try:
                    cmd,cmd_arg = text.split(" ")
                    distance = int(cmd_arg)
                    r.move(distance)
                except ValueError:
                    print "Please specify distance in whole inches."
            elif cmd == "t":
                try:
                    cmd,cmd_arg = text.split(" ")
                    heading = int(cmd_arg)
                    r.turn(heading)
                except ValueError:
                    print "Please specify turn in whole degrees."
            elif cmd == "w":
                lv.write("Lidar.dat")
            elif cmd == "h":
                print "Enter single letter command and optional argument."
        


