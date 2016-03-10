import math
import statistics as stats

class Analyzer:
    """
    Static methods grouped together.   All will probably work
    on either polar or cartesian data.
    """
    start = -10
    stop = 10
    r2_min = 0.7
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

def find_wall (array):
    """This does the stuff necessary to find the wall and the closest point on it"""
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

def avg_distance (cart_data):
    distances = map(lambda(x,y): math.sqrt(x**2+y**2), cart_data)
    return stats.mean(distances)

def aggregate_distance (ranges_at_a_heading):
    """
    Aggregate the values of the reported ranges at the distance.
    Takes a list of grouped-by range reports for the same heading
    and combines them into a single value.
    """
    ranges = [r for _,r in ranges_at_a_heading]
    return stats.mean(ranges)
    
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
    
