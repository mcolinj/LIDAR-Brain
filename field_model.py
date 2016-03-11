import itertools
import math

MM_PER_INCH = 25.4

class FakeRotation:
    """
    Like an  authentic lidar data rotation, but with fake data.
    First attempt to make it look as authentic as possible.
    Have to just fake the rpm.
    Use the field model to generate the data from the perspective
    of the robot location and orientation.
    """
    def __init__(self, field_model, robot=None, rpm=256):
        """Create the fake rotation using field model and robot orientation"""
        if robot is None:
            robot = Robot()
            
        # translate the field model origin to the robot location
        new_origin = (FieldModel.translate(x, y, robot.position) for x, y in iter(field_model))
        # convert to polar data
        tmp_data = (FieldModel.cart_to_polar(x, y) for x, y in new_origin)
        # rotate to robot orientation
        tmp_data = ((theta+robot.heading,radius) for theta,radius in tmp_data)
        # round the data to whole degrees, and whole inches (4 bits of lidar noise)
        tmp_data = itertools.starmap(FieldModel.round_degrees, tmp_data)
        # sort the data by the angle so it is in scan order
        tmp_data = sorted(tmp_data, key=lambda x: x[0])
        # then group all of the points by degree measure
        tmp_data = itertools.groupby(tmp_data, lambda x: int(round(x[0])))
        # pick the closest one in each group because it shadows the others
        self.view_data = [min(interval, key=lambda x: x[1]) for _, interval in tmp_data][::-1]
        self.origin = robot.position
        self.orientation = robot.heading
        self.rpm = rpm
        
    @staticmethod
    def polar_to_cart(theta, r):
        """convert cartesian to polar data"""
        theta_r = math.radians(theta)
        return r*math.cos(theta_r), r*math.sin(theta_r)

    def polar_data(self):
        return self.view_data

    def cartesian_data(self):
        return [FakeRotation.polar_to_cart(theta, radius)
                for theta, radius in self.polar_data()]

    def write_to_file(self, file_name):
        LidarViewer.write_to_file(file_name, polar_data)

    def rpm(self):
        return self.rpm

    def __getitem__(self, ndx):
        return self.view_data[ndx]


class Robot (object):
    """
    Models the position and the orientation of the robot.
    Robot has a position (relative to the origin in inches)
    And a heading.   move() and turn() it to update the model.
    reset() it to return it to the origin.
    """
    def __init__ (self, origin=(0,0), rotation=0):
        self.position = origin
        self.heading = rotation

    def turn(self, rotation):
        """
        Turn the robot the specified amount
        -degrees for left turn, +degrees for right turn
        """
        self.heading = self.heading + rotation

    def move(self, amount):
        """move the robot for the specified number of inches in the heading direction"""
        # make a triangle with the
        self.position = (self.position[0] + amount * math.sin(math.radians(self.heading)),
                         self.position[1] + amount * math.cos(math.radians(self.heading)))

    def reset(self):
        """Return the robot to the origin (0,0),0"""
        self.position = (0,0)
        self.heading = 0

class FieldModel (object):
    """
    Fake field cartesian coordinates.
    Accurate model of field back wall.
    Scaled up by a factor of ten to build and then scaled down
    so we have points every 10th of an inch.
    """
    # half width of field in inches * scale
    scale = 10
    field_width = 1800
    field_width_right = 600
    slot_width = 120
    slot_space = 120
    #
    # working depth of the field (12 feet or so) in inches
    field_depth = 1440
    # tower width along the back wall (4 feet, but here a 1/2 width)
    tower_width = 240
    # tower depth of the tower face
    tower_depth = math.sqrt(3)*120
    # 1/2 width of the tower face in inches
    tower_face  = 120
    # 1/2 width of the post  (this is the x projection)
    post_projection = 20
    # how far back is the wall behind the holes in the back wall
    back_back = 360
    back_width = 1800

    def __init__ (self):
        """create the cartesian equivalent for the model of the back wall of the field"""
        # left wall from corner to the left edge of the tower
        scaled_by_10_data = [(x, FieldModel.field_depth)
                           for x in xrange(-FieldModel.field_width, -FieldModel.tower_width)]

        # no need for infinity back wall behind the tower
        #scaled_by_10_data.extend((x, FieldModel.field_depth+FieldModel.back_back)
        #                         for x in range(-FieldModel.back_width, FieldModel.back_width))
        #
        # generate points for the left facet (with 16" opening)  This is 3 points on either
        # side of the goal, running along a line that is 120 degrees (with a slope of -sqrt(3)
        #
        #  Generate the points that form the posts of the left facet goal
        #  walk along the line and just put out the points that project on the
        #  first and last 2 inches.
        left_facet_delta_y = -math.sqrt(3)/2
        left_facet_delta_x = 0.5
        x = -FieldModel.tower_width
        y = FieldModel.field_depth
        while (x < -(FieldModel.tower_width/2)):
            # left post for 2 inches of x projection
            if x <= -FieldModel.tower_width+FieldModel.post_projection:
                scaled_by_10_data.append((x,y))
            # right post for 2 inches of x projection
            if x >= -(FieldModel.tower_width/2)-FieldModel.post_projection:
                scaled_by_10_data.append((x,y))
            # next point along the -120 degree line
            x = x + left_facet_delta_x
            y = y + left_facet_delta_y

        # generate the points for the face of the tower
        scaled_by_10_data.extend((x, FieldModel.field_depth-FieldModel.tower_depth)
                               for x in range(-FieldModel.tower_face, FieldModel.tower_face))

        #
        # generate points for the left facet (with 16" opening)  This is 3 points on either
        # side of the goal, running along a line that is 120 degrees (with a slope of -sqrt(3)
        #
        #  Generate the points that form the posts of the right facet goal
        #  walk along the line and just put out the points that project on the
        #  first and last 2 inches.
        left_facet_delta_y = -math.sqrt(3)/2
        left_facet_delta_x = -0.5
        x = FieldModel.tower_width
        y = FieldModel.field_depth
        while (x > FieldModel.tower_width/2):
            # left post for 2 inches of x projection
            if x >= FieldModel.tower_width-FieldModel.post_projection:
                scaled_by_10_data.append((x,y))
            # right post for 2 inches of x projection
            if x <= FieldModel.tower_width/2+FieldModel.post_projection:
                scaled_by_10_data.append((x,y))
            # next point along the -120 degree line
            x = x + left_facet_delta_x
            y = y + left_facet_delta_y
        
        slot1_start = FieldModel.tower_width+FieldModel.field_width_right
        slot1_end = slot1_start + FieldModel.slot_width
        slot2_start = slot1_end + FieldModel.slot_space
        slot2_end = slot2_start + FieldModel.slot_width
        scaled_by_10_data.extend((x, FieldModel.field_depth)
                               for x in range(FieldModel.tower_width, slot1_start))
        scaled_by_10_data.extend((x, FieldModel.field_depth)
                               for x in range(slot1_end, slot2_start))
        scaled_by_10_data.extend((x, FieldModel.field_depth)
                               for x in range(slot2_end, slot2_end+12))

        self.field_data = [(x/10.0,y/10.0) for x,y in scaled_by_10_data]

    def __getitem__ (self, index):
        return self.field_data[index]

    @staticmethod
    def tower_range_from_origin():
        return (FieldModel.field_depth - FieldModel.tower_depth) / FieldModel.scale

    @staticmethod
    def round_degrees(theta, radius):
        return int(round(theta)), radius

    @staticmethod
    def cart_to_polar(x, y):
        """convert cartesian to polar data."""
        """Map (x,y) to (theta, radius).  Theta is in degrees."""
        return math.degrees(math.atan2(y, x))-90, math.sqrt(x**2+y**2)

    @staticmethod
    def translate(x, y, origin):
        """translate the data points for the robot origin"""
        return x - origin[0], y - origin[1]

