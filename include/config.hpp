#ifndef CONFIG
#define CONFIG

#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"

//System configuration file. Include all device declarations here EXCEPT ADI devices as those do not work for some reason
//This system will likely be replaced with a HAL in the near future.

namespace sysConf
{
    //main controller
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    //motors
    pros::Motor fl = pros::Motor(10, pros::v5::MotorGears::blue);
    pros::Motor fr = pros::Motor(20, pros::v5::MotorGears::blue);
    pros::Motor rl = pros::Motor(1, pros::v5::MotorGears::blue);
    pros::Motor rr = pros::Motor(11, pros::v5::MotorGears::blue);
    pros::Motor intermedL = pros::Motor(5, pros::v5::MotorGears::blue);
    pros::Motor intermedR = pros::Motor(5, pros::v5::MotorGears::blue);
    pros::Motor roto = pros::Motor(9);
    pros::Motor arm = pros::Motor(8);

    //pros::adi::Button armStop = pros::adi::Button(0);
}

#endif