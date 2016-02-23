import math

def r_squared (*pairs):
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
    
