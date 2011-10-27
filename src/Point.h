#ifndef POINT_H
#define POINT_H

#include <base/eigen.h>

class Point
{
public:
    Point();
    base::Vector3d Xw; // points in world coordinate
    base::Vector3d Xm; // points in map coordinate
    double c; 

    //Statistics to be calcuated in Face::calculateStatistics
    float var;
    float std_dev;
};

class Point1
{
public:
    double x;
    double y;
    double z;
    double c; // reliability of this point, not utilized at the moment
};

#endif // POINT_H
