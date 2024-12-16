#ifndef CONFIG
#define CONFIG

#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"

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
