#ifndef POINT_H
#define POINT_H

#include "cml/cml.h"

class Point
{
public:
    Point();
    cml::vector3d Xw;
    cml::vector3d Xm;
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
