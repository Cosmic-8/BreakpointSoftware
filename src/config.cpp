#include "config.hpp"

//System configuration file. Include all device declarations here EXCEPT ADI devices as those do not work for some reason
//This system will likely be replaced with a HAL in the near future.

namespace sysConf
{
    //main controller
     pros::Controller master(pros::E_CONTROLLER_MASTER);

    //drive motors and configuration. ALL VALUES HERE ARE ABSOLUTELY REQUIRED
     pros::Motor L1 (11, pros::v5::MotorGear::blue, pros::v5::MotorEncoderUnits::rotations);
     pros::Motor L2 (12, pros::v5::MotorGear::blue, pros::v5::MotorEncoderUnits::rotations);
     pros::Motor L3 (13, pros::v5::MotorGear::blue, pros::v5::MotorEncoderUnits::rotations);
    
     pros::Motor R1 (14, pros::v5::MotorGear::blue, pros::v5::MotorEncoderUnits::rotations);
     pros::Motor R2 (15, pros::v5::MotorGear::blue, pros::v5::MotorEncoderUnits::rotations);
     pros::Motor R3 (16, pros::v5::MotorGear::blue, pros::v5::MotorEncoderUnits::rotations);

     pros::IMU imu (5);
     pros::Rotation rearEncoder(20);

    //-------------------
    //values for odometry
    //-------------------
     const float wheelDiam = 4;
     const float driveRatio = 48/72;
     const float rearEncoderWheelDiam = 2;

    //offsets from COR for encoder ground contact
     const float leftOffset = 7.5;
     const float rightOffset = 7.5;
     const float rearOffset = 4;
}