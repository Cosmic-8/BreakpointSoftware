#ifndef CONFIG
#define CONFIG

#define _generalDebug false
#define _PIDdebug false
#define _odometryLogging false
#define _showOdomMath false

#include "api.h"

//System configuration file. Include all device declarations here EXCEPT ADI devices as those do not work for some reason
//This system will likely be replaced with a HAL in the near future.

namespace sysConf
{
    //main controller
    extern pros::Controller master;

    //drive motors and configuration. ALL VALUES HERE ARE ABSOLUTELY REQUIRED
    extern pros::Motor L1;
    extern pros::Motor L2;
    extern pros::Motor L3;
    extern pros::Motor R1;
    extern pros::Motor R2;
    extern pros::Motor R3;

    extern pros::IMU imu;
    extern pros::Rotation rearEncoder;

    //-------------------
    //values for odometry
    //-------------------
    extern const float wheelDiam;
    extern const float driveRatio;
    extern const float rearEncoderWheelDiam;

    //offsets from COR for encoder ground contact
    extern const float leftOffset;
    extern const float rightOffset;
    extern const float rearOffset;

    //pros::adi::Button armStop = pros::adi::Button(0);
}

#endif