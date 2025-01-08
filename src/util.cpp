#include "util.hpp"

//* --- functions ------------------------------------
namespace MatrixFoundations {

    double centidegreesToRotations(double centidegrees) //* centidegrees to rotations
    {
        double valueInRotations = (centidegrees / 100) / 360;
        return valueInRotations;
    }

    int sign(double number) //* returns sign (-1, 0, or 1)
    {
        if (number >= 0) {
            return 1;
        } else {
            return -1;
        }
    }

    double wrapDegrees(double angle) //* wrap degrees -> (-360, 360)
    {
        double wrappedAngle = angle;
        while (wrappedAngle >= 360) {
            wrappedAngle -= 360;
        }
        while (wrappedAngle <= -360) {
            wrappedAngle += 360;
        }
        return wrappedAngle;
    }

    double wrapRadians(double angle) //* wrap radians -> (-2pi, 2pi)
    {
        double wrappedAngle = angle;
        while (wrappedAngle >= 2 * M_PI) {
            wrappedAngle -= 2 * M_PI;
        }
        while (wrappedAngle <= -2 * M_PI) {
            wrappedAngle += 2 * M_PI;
        }
        return wrappedAngle;
    }

    double radiansToDegrees(double angle) //* radians to degrees
    {
        return angle * 180 / M_PI;
    }

    double degreesToRadians(double angle) //* degrees to radians
    {
        return angle * M_PI / 180;
    }

    double minutesToMilliseconds(double minutes) {
        return minutes * 60 * 1000;
    }

    double millisecondsToMinutes(double milliseconds) {
        return (milliseconds / 60) / 1000;
    }


    double getDistance(std::vector<double> firstPoint, std::vector<double> secondPoint) {
        return sqrt(pow(firstPoint[0] - secondPoint[0], 2) + pow(firstPoint[1] + secondPoint[1], 2));
    }

    float sum(int i, float data[]) 
    {
        int loopIndex = i;
        float sum = 0;

        while(loopIndex != 0)
        {
            sum = sum + data[loopIndex];
            loopIndex --;
        }
        return sum;
    }

    float average(int i, float data[])
    {
        return sum(i, data) / i;
    }
}
//* --------------------------------------------------------

bool odometryLogging = false;   //* prints odometry values
bool autonSelectorLogs = false; //* prints auton choices
bool showOdomMath = false;      //* prints odometry math
bool purePursuitLogs = false;   //* prints pure pursuit goal points
bool purePursuitMath = false;   //* prints pure pursuit math
bool ramseteLogs = false;
bool ramseteMath = false;
bool generalDebug = false; //* e.g., "turned to x degrees"
bool velocityLogging = false;
bool PIDdebug = false;