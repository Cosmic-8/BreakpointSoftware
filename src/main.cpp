#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "teleop.hpp"
#include "config.hpp"
#include <cerrno>
#include <cstdio>
#include <iostream>

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::print(0, "Breakpoint Holonomic Control V1");
	std::cout << "Initialize" << std::endl;
	std::cout << errno << std::endl;
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	// pros::Controller master(pros::E_CONTROLLER_MASTER);
	// pros::Motor 
	pros::adi::Button armStop = pros::adi::Button(0);
	TeleoperationSystem teleop(sysConf::fl, sysConf::fr, sysConf::rl, sysConf::rr, sysConf::master, 1);
	teleop.Start();

	bool rotorRunning = false; //for flip flop controller for flywheel control
	sysConf::arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	//std::cout << "entering main control loop" << std::endl;
	while (true) {
		teleop.Update(); //update teleop loop;
		std::cout << armStop.get_value() << std::endl;
		if(sysConf::master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && armStop.get_value())
		{
			sysConf::arm.move_voltage(9000);
		}
		else if(sysConf::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			sysConf::arm.move_voltage(-9000);
		}
		else
		{
			sysConf::arm.brake();
		}

		if(sysConf::master.get_digital_new_press(DIGITAL_R2))
		{
			if(rotorRunning == true)
			{
				rotorRunning = false;
				sysConf::roto.brake();
			}
			else 
			{
				rotorRunning = true;
				sysConf::roto.move_voltage(INT_MAX);
			}
		}

		pros::delay(20); //avoid system resource drain
	}
	std::cout << "Program end" << std::endl;
}