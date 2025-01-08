#include "api.h"

#ifndef UTIL_HPP
#define UTIL_HPP

//* ----------------------------------------

extern int autonNum;

extern bool keepPositionTask;

namespace util
{
    double centidegreesToRotations(double centidegrees);
    int sign(double number);
    double wrapDegrees(double angle);
    double wrapRadians(double angle);
    double radiansToDegrees(double angle);
    double degreesToRadians(double angle);
    double minutesToMilliseconds(double minutes);
    double millisecondsToMinutes(double milliseconds);
    double getDistance(std::vector<double> firstPoint, std::vector<double> secondPoint);
    float average(int i, float data[]);
    float sum(int i, float data[]);
}

#endif