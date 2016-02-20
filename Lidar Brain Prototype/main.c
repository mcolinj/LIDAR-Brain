//
//  main.c
//  Lidar Brain Prototype
//
//  Created by Charles Monk on 1/31/16.
//  Copyright © 2016 Charles Monk. All rights reserved.
//

#include <stdio.h>
#include <tgmath.h>
#include <math.h>
double r2_min = 0.9;
double slopes[360];
int slopeCount = 0;

typedef struct myRect_r {
    double x,y;
} myRect;

myRect *toRect (double degree, double distance, myRect *orderedPair) {
    //Convert to radians
    degree *= M_PI/180.0;
    //Convert polar coordinates to rectangular coordinates and return an array with the x-value at index 0 and the y-value at index 1
    orderedPair->x = distance * cos(degree);
    orderedPair->y= distance * sin(degree);
    return orderedPair;
}

double r_slope (myRect *pair1, myRect *pair2, myRect *pair3, myRect *pair4) {
    double numerator1 = 4*(pair1->x*pair1->y + pair2->x*pair2->y + pair3->x*pair3->y + pair4->x*pair4->y);
    double numerator2 = (pair1->x+pair2->x+pair3->x+pair4->x) * (pair1->y+pair2->y+pair3->y+pair4->y);
    
    double denominator1 = 4*( pair1->x*pair1->x + pair2->x*pair2->x + pair3->x*pair3->x + pair4->x*pair4->x );
    double denominator2 = (pair1->x + pair2->x + pair3->x + pair4->x) * (pair1->x + pair2->x + pair3->x + pair4->x);
    
    double numerator = numerator1 * numerator2;
    double denominator = denominator1 * denominator2;
    
    double slope = numerator/denominator;
    
    slopes[slopeCount] = slope;
    slopeCount++;
    
    return slope;
}

//void findParallel () {
//    double dividends[slopeCount];
//    double parallelLines[slopeCount];
//    
//    for (int i = 0; i < slopeCount - 1; i++) {
//        dividends[i] = slopes[i] / slopes[i+1];
//    }
//    
//    for (int i = 0; i < sizeof(slopes)/sizeof(double); i++) {
//        int v = 0;
//        
//        if ( dividends[i] < 1.2 && dividends[i] > -1.2 && (dividends[i] > 0.0001 || dividends[i] < -0.0001) ) {
//            parallelLines[v] = dividends[i];
//            printf("The ratio is %f \n", parallelLines[v]);
//            
//            v++;
//        }
//    }
//}

double r_squared (myRect *pair1, myRect *pair2, myRect *pair3, myRect *pair4){
    //This calculates r-squared for 4 points
    //I split the equation into 3 variables to minimize risk of a mistake
    
    double numerator = 4*(pair1->x*pair1->y + pair2->x*pair2->y + pair3->x*pair3->y + pair4->x*pair4->y)
    - ( (pair1->x + pair2->x + pair3->x + pair4->x) * (pair1->y + pair2->y + pair3->y + pair4->y) );
    
    double denominator1 = sqrt(  4*(pair1->x*pair1->x + pair2->x*pair2->x + pair3->x*pair3->x + pair4->x*pair4->x)
    - ( (pair1->x + pair2->x + pair3->x + pair4->x) * (pair1->x + pair2->x + pair3->x + pair4->x) )  );
    
    double denominator2 = sqrt(  4*(pair1->y*pair1->y + pair2->y*pair2->y + pair3->y*pair3->y + pair4->y*pair4->y)
    - ( (pair1->y + pair2->y + pair3->y + pair4->y) * (pair1->y + pair2->y + pair3->y + pair4->y) )  );
    
    if (denominator1*denominator2 != 0) {
        double combination1 = numerator / ( denominator1 * denominator2 );
        
        double combination2 = combination1 * combination1;
        
        return combination2;
    } else {
        printf("There was a denominator of zero");
        return 0.0;
    }
    
}

// here you are copying into another array, so you either need to pass it
int removeJunk (double *withJunk, double size, double *withoutJunk){
    
    static int indexNumber = 0;
    
    for (int i=0; i < ( size ); i++) {
        if (withJunk[i] != 590.6) {
            withoutJunk[indexNumber] = withJunk[i];
            indexNumber++;
        }
    }
    return indexNumber;
}

double magnitudeChecker (int i,int d, myRect *array) {
    double magnitude;
    double r2 = r_squared(&array[i], &array[d+1], &array[d+2], &array[d+3]);
    if (r2 < r2_min) {
        myRect smallest = array[i];
        myRect largest = array[d+3];
        magnitude = sqrt( (largest.x - smallest.x)*(largest.x - smallest.x) + (largest.y - smallest.y)*(largest.y - smallest.y) );
        printf("The magnitude is %f ((%f,%f) = smallest, (%f, %f) = largest) \n", magnitude, smallest.x, smallest.y, largest.x, largest.y);
        //r_slope(&array[i], &array[d+1], &array[d+2], &array[d+3]);
        return magnitude;
    } else {
        return magnitudeChecker(i, d+1, array);
    }
}

int main(int argc, const char * argv[]) {
    //Array of the data points
    double data[360] = {590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,22.1,590.6,
        21.1,590.6,19.6,590.6,18.9,590.6,18.7,590.6,18.9,590.6,28.4,590.6,28.4,590.6,85.1,590.6,83.7,590.6,218.9,590.6,187.8,590.6,187.5,590.6,590.6,590.6,590.6,590.6,590.6,590.6,
        590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6
        ,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,214.3,590.6,130.4,590.6,590.6,590.6,590.6,590.6,215.9,590.6,222.7,590.6,223.9,590.6,590.6,590.6,590.6,
        590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,
        590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,40.6,590.6,41.7,590.6,42.9,590.6,71.1,590.6,60.4,590.6,53.9,590.6,49.6,590.6,49.3,590.6,49.0,590.6,89.8,
        590.6,83.7,590.6,78.0,590.6,73.4,590.6,69.1,590.6,65.4,590.6,62.2,590.6,59.3,590.6,56.9,590.6,49.5,590.6,49.9,590.6,50.7,590.6,49.1,590.6,47.6,590.6,46.3,590.6,45.0,590.6,
        43.9,590.6,42.9,590.6,42.0,590.6,41.2,590.6,40.4,590.6,39.7,590.6,39.2,590.6,38.7,590.6,38.1,590.6,37.7,590.6,37.4,590.6,37.0,590.6,36.8,590.6,36.6,590.6,36.5,590.6,36.3,
        590.6,36.2,590.6,36.2,590.6,36.2,590.6,36.3,590.6,36.4,590.6,36.5,590.6,36.7,590.6,37.0,590.6,37.3,590.6,37.7,590.6,38.3,590.6,38.9,590.6,590.6,590.6,590.6,590.6,590.6,590.6
        ,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,
        590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6
        ,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,
        590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6};
    

     //int goodDataSize = removeJunk(data, sizeof(data), withoutJunk);
    
    /* we need a rectangular coordinate pair for each polar point */
    myRect OrderedPairs[360];
    
    /* convert all of the radial points to rectangular coordinates */
    int goodPoints = 0;
    for (int i = 0; i < 360; i++) {
        if (data[i] != 590.6) {
            toRect(i, data[i], &OrderedPairs[goodPoints]);
            goodPoints++;
        }
    }
    
//    //Start test to ensure it's working properly (don't uncomment unless testing)
//    myRect OrderedPairs[] = { { -81.0, 144.0 },
//        { -72.0, 144.0 },
//        { -63.0, 143.0 },
//        { -50.0, 145.0 },
//        { -40.0, 142.0 },
//        { -30.0, 147.0 },
//        { -5.0, 144.0 },
//        { 0.0, 142.0 },
//        { 12.0, 148.0 },
//        { 24.0, 144.0 },
//        { 36.0, 142.0 },
//        { 36.0, 138.0 },
//        { 40.0, 128.0 },
//        { 48.0, 120.0 },
//        { 50.0, 120.0 },
//        { 60.0, 120.0 } };
//
//    int goodPoints = 16;
//    //End test
    
    double magnitudes[360];
    int magnitudeCount = 0;
    
    myRect greatestMagnitudePoints[4];
    double greatestMagnitude = magnitudes[0];
    
    myRect frontFaceOfTowerPoints[4];
    
    for (int i = 0; i < goodPoints - 3; i++) {
        //Run through the array to find sets of 4 colinear points (and filtering out data garbage)
        
        double rs;
        
        if( (rs = r_squared(&OrderedPairs[i], &OrderedPairs[i+1], &OrderedPairs[i+2], &OrderedPairs[i+3]) > r2_min)) {
            printf("%f \n", rs);
            magnitudes[magnitudeCount] = magnitudeChecker(i, i, OrderedPairs);
            magnitudeCount++;
            if (magnitudes[magnitudeCount] > greatestMagnitude) {
                
                greatestMagnitude = magnitudes[magnitudeCount];
                
                greatestMagnitudePoints[0] = OrderedPairs[i];
                greatestMagnitudePoints[1] = OrderedPairs[i+1];
                greatestMagnitudePoints[2] = OrderedPairs[i+2];
                greatestMagnitudePoints[3] = OrderedPairs[i+3];
                
                printf("Wall found with points (%f, %f), (%f, %f), (%f, %f), (%f, %f)", greatestMagnitudePoints[0].x, greatestMagnitudePoints[0].y, greatestMagnitudePoints[1].x,greatestMagnitudePoints[1].y, greatestMagnitudePoints[2].x, greatestMagnitudePoints[2].y, greatestMagnitudePoints[3].x, greatestMagnitudePoints[3].y);
            }
            //To be used if the robot is in a position that the front face of the tower can be found (we won't be using it, as our robot will go under the low bar
//            } else if (magnitudes[magnitudeCount] > 20  &&  magnitudes[magnitudeCount] < 28) {
//                frontFaceOfTowerPoints[0] = OrderedPairs[i];
//                frontFaceOfTowerPoints[1] = OrderedPairs[i+1];
//                frontFaceOfTowerPoints[2] = OrderedPairs[i+2];
//                frontFaceOfTowerPoints[3] = OrderedPairs[i+3];
//                
//                printf("Front face found");
//            }
                
        } else {
            printf("Not found %f = r_squared([%f,%f]) \n", rs, OrderedPairs[i].x, OrderedPairs[i].y);
        }
        
    }
    //findParallel();
    
    double closestPoint = (greatestMagnitudePoints[0].x)*(greatestMagnitudePoints[0].x) + (greatestMagnitudePoints[0].y)*(greatestMagnitudePoints[0].y);
    for (int i = 0; i < 4; i++) {
        if (((greatestMagnitudePoints[i].x)*(greatestMagnitudePoints[i].x) + (greatestMagnitudePoints[i].y)*(greatestMagnitudePoints[i].y)) < closestPoint) {
            closestPoint = (greatestMagnitudePoints[i].x)*(greatestMagnitudePoints[i].x) + (greatestMagnitudePoints[i].y)*(greatestMagnitudePoints[i].y);
        }
    }

    //Find the magnitude approximatly equal to 2 feet, as this is the face of the tower parallel to the wall
    
    
    
    
//In case testing of regression formula is required (is commented because not currently needed):

//    myRect r1 = {1.0, 1.0};
//    myRect r2 = {2.0, 2.0};
//    myRect r3 = {3.0, 3.0};
//    myRect r4 = {4.0, 4.0};
//
//    double rs;
//    if((rs = r_squared(&r1,&r2,&r3,&r4)) > r2_min) {
//        printf("IT'S ALIVE, IT'S ALIVE!!!!!!!!! (with the value of %f) \n", rs);
//    } else {
//        printf("not found %f \n", rs);
//    }

    return 0;
}