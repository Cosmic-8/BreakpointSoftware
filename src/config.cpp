#include "config.hpp"

//System configuration file. Include all device declarations here EXCEPT ADI devices as those do not work for some reason
//This system will likely be replaced with a HAL in the near future.

namespace sysConf
{
    //main controller
     pros::Controller master(pros::E_CONTROLLER_MASTER);

    //drive motors and configuration. ALL VALUES HERE ARE ABSOLUTELY REQUIRED
     pros::Motor L1 (1, pros::v5::MotorGear::blue, pros::v5::MotorEncoderUnits::rotations);
     pros::Motor L2 (2, pros::v5::MotorGear::blue, pros::v5::MotorEncoderUnits::rotations);
     pros::Motor L3 (3, pros::v5::MotorGear::blue, pros::v5::MotorEncoderUnits::rotations);
    
     pros::Motor R1 (4, pros::v5::MotorGear::blue, pros::v5::MotorEncoderUnits::rotations);
     pros::Motor R2 (5, pros::v5::MotorGear::blue, pros::v5::MotorEncoderUnits::rotations);
     pros::Motor R3 (6, pros::v5::MotorGear::blue, pros::v5::MotorEncoderUnits::rotations);

     pros::IMU imu (11);
     pros::Rotation rearEncoder(14);

    //-------------------
    //values for odometry
    //-------------------
     const float wheelDiam = 4;
     const float driveRatio = 48/72;
     const float rearEncoderWheelDiam = 2;

    //offsets from COR for encoder ground contact
     const float leftOffset = 1;
     const float rightOffset = 1;
     const float rearOffset = 1;

    //pros::adi::Button armStop = pros::adi::Button(0);
}