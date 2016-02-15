import numpy as np
import pylab
import matplotlib.pyplot as plt
import itertools
import math
import pdb

MM_PER_INCH = 25.4

class LidarView (object):
    begin_marker = 888
    end_marker = 999
    fig = pylab.figure(figsize=(9,9))
    ax = fig.add_subplot(111, projection='polar')
    ax.set_rmax(240)
    ax.grid(True)

    @staticmethod
    def round_values(r, theta):
        return int(round(r)), int(round(theta))
    
    def __init__ (self, polar_data, robot):
        """create lidar view from a set of rotated/translated polar points in the model world"""
        # round the data to whole degrees, and whole inches (4 bits of lidar noise)
        polar_data = itertools.starmap(LidarView.round_values, polar_data)
        # sort the data by the angle so it is in scan order
        polar_data = sorted(polar_data, key=lambda x: x[1])
        # then group all of the points by degree measure
        polar_data = itertools.groupby(polar_data, lambda x: int(round(x[1])))
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
        pylab.show(block=False)
    def write (self, fname):
        with open(fname, "a") as text_file:
            text_file.write("{:d}, {:d}\n".format(LidarView.begin_marker,
                                                  len(self.data)))
            for r,theta in self.data:
                text_file.write("{:f.1}, {:f.1}\n".format(theta, r))

            text_file.write("{:d}, {:f.1} ({:f.1},{:f.1})\n".format(LidarView.begin_marker,
                                                                    self.orientation,
                                                                    self.origin[0],
                                                                    self.origin[1]))

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
    field_width_right = 60
    slot_width = 12
    slot_space = 12
    #
    # working depth of the field (12 feet or so) in inches
    field_depth = 144
    # tower width along the back wall (4 feet, but here a 1/2 width)
    tower_width = 24
    # tower depth of the tower face
    tower_depth = math.sqrt(3)*12
    # 1/2 width of the tower face in inches
    tower_face  = 12
    # 1/2 width of the post  (this is the x projection)
    post_projection = 2
    # how far back is the wall behind the holes in the back wall
    back_back = 36
    back_width = 180
    def __init__ (self):
        """create the cartesian equivalent for the model of the back wall of the field"""
        # left wall from corner to the left edge of the tower
        self.field_data = [(x, Field.field_depth)
                           for x in xrange(-Field.field_width, -Field.tower_width)]

        # set up an infinity back wall behind the tower
        self.field_data.extend((x, Field.field_depth+Field.back_back)
                               for x in range(-Field.back_width, Field.back_width))
        #
        # generate points for the left facet (with 16" opening)  This is 3 points on either
        # side of the goal, running along a line that is 120 degrees (with a slope of -sqrt(3)
        #
        #
        #  Generate the points that form the posts of the left facet goal
        #  walk along the line and just put out the points that project on the
        #  first and last 2 inches.
        left_facet_delta_y = -math.sqrt(3)/2
        left_facet_delta_x = 0.5
        x = -Field.tower_width
        y = Field.field_depth
        while (x < -(Field.tower_width/2)):
            # left post for 2 inches of x projection
            if x <= -Field.tower_width+Field.post_projection:
                self.field_data.append((x,y))
            # right post for 2 inches of x projection
            if x >= -(Field.tower_width/2)-Field.post_projection:
                self.field_data.append((x,y))
            # next point along the -120 degree line
            x = x + left_facet_delta_x
            y = y + left_facet_delta_y

        # generate the points for the face of the tower
        self.field_data.extend((x, Field.field_depth-Field.tower_depth)
                               for x in range(-Field.tower_face, Field.tower_face))

        #
        # generate points for the left facet (with 16" opening)  This is 3 points on either
        # side of the goal, running along a line that is 120 degrees (with a slope of -sqrt(3)
        #
        #
        #  Generate the points that form the posts of the right facet goal
        #  walk along the line and just put out the points that project on the
        #  first and last 2 inches.
        left_facet_delta_y = -math.sqrt(3)/2
        left_facet_delta_x = -0.5
        x = Field.tower_width
        y = Field.field_depth
        while (x > Field.tower_width/2):
            # left post for 2 inches of x projection
            if x >= Field.tower_width-Field.post_projection:
                self.field_data.append((x,y))
            # right post for 2 inches of x projection
            if x <= Field.tower_width/2+Field.post_projection:
                self.field_data.append((x,y))
            # next point along the -120 degree line
            x = x + left_facet_delta_x
            y = y + left_facet_delta_y
        
        slot1_start = Field.tower_width+Field.field_width_right
        slot1_end = slot1_start + 12
        slot2_start = slot1_end + 12
        slot2_end = slot2_start + 12
        self.field_data.extend((x, Field.field_depth)
                               for x in range(Field.tower_width, slot1_start))
        self.field_data.extend((x, Field.field_depth)
                               for x in range(slot1_end, slot2_start))
        self.field_data.extend((x, Field.field_depth)
                               for x in range(slot2_end, slot2_end+12))

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
        self.fig = pylab.figure(figsize=(9,9))
        self.axes = self.fig.add_subplot(111,aspect='equal')
        self.axes.set_autoscale_on(False)
        self.axes.set_xlim([-Field.field_width,Field.field_width])
        self.axes.set_ylim([-Field.field_width,Field.field_width])

        x, y  = zip(*self.field_data)

        self.axes.plot(x, y, 'x')
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
                    lv = f.createLidarView(r)
                    lv.plot()
                except ValueError:
                    print "Please specify distance in whole inches."
            elif cmd == "t":
                try:
                    cmd,cmd_arg = text.split(" ")
                    heading = int(cmd_arg)
                    r.turn(heading)
                    lv = f.createLidarView(r)
                    lv.plot()
                except ValueError:
                    print "Please specify turn in whole degrees."
            elif cmd == "w":
                lv.write("Lidar.dat")
            elif cmd == "h":
                print "Enter single letter command and optional argument."
        


