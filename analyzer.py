import math
#import statistics as stats

class Analyzer:
    """
    Static methods grouped together.   All will probably work
    on either polar or cartesian data.
    """
    start = -10
    stop = 10
    r2_min = 0.80
    #
    #
    #
    @staticmethod
    def range_at_heading(polar_data, sweep):
        """
        Find the closest hit at a heading in the sweep.
        Return the range and the heading as a tuple.
        (heading, range)
        """
        min_reading = (0, 0)
        sweep_data = [x for x in polar_data if x[0] in range(*sweep)]
        
        if sweep_data:
            min_reading = min(sweep_data, key=lambda r:r[1])
            
        return min_reading

def r_squared (pairs):
    #This calculates r-squared for 4 points
    #I split the equation into 3 variables to minimize risk of a mistake
    length = len(pairs)
    numerator = length*sum(x*y for x,y in pairs) - sum(x for x,_ in pairs)*sum(y for _,y in pairs)
    denominator1 = math.sqrt( length*sum(x**2 for x,_ in pairs) - sum(x for x,_ in pairs)**2 )
    denominator2 = math.sqrt( length*sum(y**2 for _,y in pairs) - sum(y for _,y in pairs)**2 )
    
    if denominator1*denominator2 != 0:
    	combination1 = float(numerator) / float(denominator1*denominator2)
    	combination2 = combination1**2
    	return combination2
    else:
    	print "There was a denominator of zero"
    	return 0.0

def magnitude_checker (i, d, array):
    """This finds the magnitude and the closest point to the robot"""
    i2d = array[i:d]
    r2 = r_squared(i2d)
    if (r2 < Analyzer.r2_min) or (d >= len(array)-1):
        closest_point = (999, 999)
        smallest = i2d[0]
        largest = i2d[-1]
        magnitude = math.sqrt( (largest[0] - smallest[0])**2 + (largest[1] - smallest[1])**2 )
        for x,y in i2d:
            if (  x**2 + y**2 < closest_point[0]**2 + closest_point[1]**2 ):
                closest_point = (x, y)
        return (magnitude, closest_point)
    else:
        return magnitude_checker(i, d+1, array)

def magnitude_checker_midpoint (i, d, array):
    """
    This finds the magnitude and the midpoint and slope of vector along the wall.   Returns them as a tuple.   Slope is returned as a tuple (rise,run).
    """
    i2d = array[i:d]
    r2 = r_squared(i2d)
    if (r2 < Analyzer.r2_min) or (d >= len(array)-1):
        closest_point = (999, 999)
        smallest = i2d[0]
        largest = i2d[-1]
        magnitude = math.sqrt( (largest[0] - smallest[0])**2 + (largest[1] - smallest[1])**2 )

        for x,y in i2d:
            if (  x**2 + y**2 < closest_point[0]**2 + closest_point[1]**2 ):
                closest_point = (x, y)

        #  midpoint calculation is just average of the points
        midpoint_x = (smallest[0]+largest[0])/2.0
        midpoint_y = (smallest[1]+largest[1])/2.0
        midpoint = (midpoint_x, midpoint_y)
        slope = (largest[1]-smallest[1],largest[0]-smallest[0])
        return (magnitude, slope, midpoint)
    else:
        return magnitude_checker_midpoint(i, d+1, array)

def find_wall (array):
    """
    This does the stuff necessary to find the wall and the closest point on it.
    """
    magnitudes_and_points = []
    for i in range(0, len(array)-3):
    # Since magnitudeChecker returns the magnitude and closest point, this puts the former at index 0 and the latter at index 1
        magnitudes_and_points.append( magnitude_checker(i, i+4, array) )

        (mag, point) = magnitudes_and_points[0]
        for m,p in magnitudes_and_points:
            if m > mag:
                mag = m
                point = p

    return (mag, point)


def find_wall_midpoint (array):
    """
    This does the stuff necessary to find the wall and the mid point of it.
    Return the heading and the range from robot to the point.
    Also return the orientation of the wall with respect to the robot.   This is the angle of the normal to the wall.  i.e. if the robot turned by "orientation" amount, it could head straight to the wall.
    """
    magnitudes_and_points = []
    for i in range(0, len(array)-3):
    # Since magnitudeChecker returns the magnitude and closest point, this puts the former at index 0 and the latter at index 1
        magnitudes_and_points.append( magnitude_checker_midpoint(i, i+4, array) )

        (mag, slope, midpoint) = magnitudes_and_points[0]
        for m,s,p in magnitudes_and_points:
            if m > mag:
                mag = m
                slope = s
                midpoint = p

    if (len(magnitudes_and_points) > 1):
        distance = math.sqrt(midpoint[0]**2 + midpoint[1]**2)
        heading = -math.degrees(math.atan2(midpoint[1],midpoint[0]))
        orientation = -math.degrees(math.atan2(slope[1],slope[0]))-90
        if orientation <= -180:
            orientation = orientation + 360

        return heading, distance, orientation
    else:
        return 0,0,0


#def avg_distance (cart_data):
#    distances = map(lambda(x,y): math.sqrt(x**2+y**2), cart_data)
#    return stats.mean(distances)

#def aggregate_distance (ranges_at_a_heading):
#    """
#    Aggregate the values of the reported ranges at the distance.
#    Takes a list of grouped-by range reports for the same heading
#    and combines them into a single value.
#    """
#    ranges = [r for _,r in ranges_at_a_heading]
#    return stats.mean(ranges)
    
    # double numerator = 4*(pair1->x*pair1->y + pair2->x*pair2->y + pair3->x*pair3->y + pair4->x*pair4->y)
#     - ( (pair1->x + pair2->x + pair3->x + pair4->x) * (pair1->y + pair2->y + pair3->y + pair4->y) );
#     
#     double denominator1 = sqrt(  4*(pair1->x*pair1->x + pair2->x*pair2->x + pair3->x*pair3->x + pair4->x*pair4->x)
#     - ( (pair1->x + pair2->x + pair3->x + pair4->x) * (pair1->x + pair2->x + pair3->x + pair4->x) )  );
#     
#     double denominator2 = sqrt(  4*(pair1->y*pair1->y + pair2->y*pair2->y + pair3->y*pair3->y + pair4->y*pair4->y)
#     - ( (pair1->y + pair2->y + pair3->y + pair4->y) * (pair1->y + pair2->y + pair3->y + pair4->y) )  );
#     
#     if (denominator1*denominator2 != 0) {
#         double combination1 = numerator / ( denominator1 * denominator2 );
#         
#         double combination2 = combination1 * combination1;
#         
#         return combination2;
#     } else {
#         printf("There was a denominator of zero");
#         return 0.0;
#     }

def something_new(cart_data):
    r2_markers = []
    building_vector_start = None
    building_vector_end = None
    max_mag = 0
    for i in range(0, len(cart_data)-factor):
        slice = cart_data[i:i+factor]
        r2 = r_squared(slice)
        # start or continue
        if r2 > .7:
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

    
