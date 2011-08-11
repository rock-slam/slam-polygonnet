#ifndef POINT_H
#define POINT_H

#include "base/eigen.h"

class Point
{
public:
    Point();
    base::Vector3d Xw;
    base::Vector3d Xm;
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
    double c;
};

#endif // POINT_H
