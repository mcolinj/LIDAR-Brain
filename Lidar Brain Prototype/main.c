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

void findParallel () {
    double dividends[slopeCount + 1];
    double parallelLines[slopeCount + 1];
    
    for (int i = 0; i < slopeCount - 1; i++) {
        dividends[i] = slopes[i] / slopes[i+1];
    }
    
    for (int i = 0; i < sizeof(slopes)/sizeof(double); i++) {
        int v = 0;
        
        if ( dividends[i] < 1.2 && dividends[i] > -1.2 && (dividends[i] > 0.0001 || dividends[i] < -0.0001) ) {
            parallelLines[v] = dividends[i];
            printf("The ratio is %f \n", parallelLines[v]);
            
            v++;
        }
    }
}

double r_squared (myRect *pair1, myRect *pair2, myRect *pair3, myRect *pair4){
    //This calculates r-squared for 4 points
    //I split the equation into 3 variables to minimize risk of a mistake
    
    double numerator = 4*(pair1->x*pair1->y + pair2->x*pair2->y + pair3->x*pair3->y + pair4->x*pair4->y)
    - ( (pair1->x + pair2->x + pair3->x + pair4->x) * (pair1->y + pair2->y + pair3->y + pair4->y) );
    
#warning don't allow denominator of zero
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
        return magnitude;
    } else {
        return magnitudeChecker(i, d+1, array);
    }
}

int main(int argc, const char * argv[]) {
    //Array of the data points
    double data[360] = {30.7,590.6,590.6,590.6,19.1,590.6,19.1,590.6,590.6,590.6,29.8,590.6,29.0,590.6,28.8,590.6,29.3,590.6,30.6,590.6,57.0,
        590.6,58.9,590.6,49.6,590.6,49.0,590.6,48.7,590.6,50.7,590.6,37.6,590.6,37.0,590.6,47.5,590.6,47.3,590.6,47.2,590.6,47.2,590.6,590.6,
        590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,172.8,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,
        590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,191.9,590.6,590.6,590.6,187.6,
        590.6,215.7,590.6,590.6,590.6,590.6,590.6,590.6,590.6,211.2,590.6,220.7,590.6,220.0,590.6,590.6,590.6,590.6,590.6,590.6,590.6,
        590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,
        590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,590.6,48.3,590.6,
        39.3,590.6,36.4,590.6,36.1,590.6,35.7,590.6,92.0,590.6,75.0,590.6,76.1,590.6,69.6,590.6,64.5,590.6,60.2,590.6,56.8,590.6,53.5,590.6,
        50.7,590.6,48.3,590.6,46.1,590.6,44.1,590.6,42.3,590.6,35.5,590.6,35.8,590.6,36.2,590.6,36.9,590.6,35.8,590.6,34.8,590.6,34.1,590.6,
        33.2,590.6,32.5,590.6,31.9,590.6,31.2,590.6,30.7,590.6,30.3,590.6,29.8,590.6,29.5,590.6,29.1,590.6,28.8,590.6,28.5,590.6,28.3,590.6,
        28.2,590.6,28.1,590.6,28.0,590.6,27.9,590.6,27.9,590.6,27.9,590.6,27.9,590.6,28.0,590.6,28.1,590.6,28.2,590.6,28.5,590.6,28.7,590.6,
        28.9,590.6,29.3,590.6,29.6,590.6,30.0,590.6,30.5,590.6,30.9,590.6,31.5,590.6,32.2,590.6,32.8,590.6,33.6,590.6,34.5,590.6,35.4,590.6,
        36.5,590.6,37.6,590.6,38.9,590.6,40.4,590.6,41.9,590.6,43.6,590.6,45.7,590.6,48.0,590.6,50.3,590.6,53.5,590.6,57.1,590.6,61.2,590.6,
        65.2,590.6,66.1,590.6,63.7,590.6,61.2,590.6,59.3,590.6,57.6,590.6,56.2,590.6,54.6,590.6,53.6,590.6,52.4,590.6,51.9,590.6,50.7,590.6,
        50.2,590.6,49.4,590.6,49.1,590.6,48.3,590.6,48.0,590.6,47.6,590.6,47.2,590.6,46.7,590.6,46.5,590.6,46.3,590.6,46.2,590.6,46.1,590.6,
        46.1,590.6,46.2,590.6,46.3,590.6,46.8,590.6,47.0,590.6,590.6,590.6,31.5,590.6};
    

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
    
    
    for (int i = 0; i < goodPoints - 3; i++) {
        //Run through the array to find sets of 4 colinear points (and filtering out data garbage)
        
        double rs;
        
        if( (rs = r_squared(&OrderedPairs[i], &OrderedPairs[i+1], &OrderedPairs[i+2], &OrderedPairs[i+3]) > r2_min)) {
            printf("%f \n", rs);
            magnitudeChecker(i, i, OrderedPairs);
            
        } else {
            printf("Not found %f = r_squared([%f,%f]) \n", rs, OrderedPairs[i].x, OrderedPairs[i].y);
        }
        
    }
    findParallel();
    
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
