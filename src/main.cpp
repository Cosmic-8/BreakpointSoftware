#include "main.h"
#include "config.hpp"
#include "pros/rtos.hpp"
#include "teleop.hpp"
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::delay(500);
	pros::lcd::print(1, "Calibrating IMU");
	sysConf::imu.reset(true);

	pros::lcd::print(2, "IMU Calibration Complete");
	sysConf::master.clear();
	sysConf::master.print(0, 1, "System Ready");


	std::cout << "Attempting Odom Init" << std::endl;
	// //startup odometry subsystem
	// softwareSubsystems::odometryV2* odom = softwareSubsystems::odometryV2::getInstance();
	
	// odom->resetAll();
	// odom->setup(0,0,0); //set starting location/offset here

	// odom->trackPosition(); //start background process
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task wilprosl exit.
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
void autonomous()
{
	// hopefully this doesn't break anything

	// testing
	drivePID driveCtl;
	driveCtl.setPIDValues(1, 0, 0);		// TUNE HERE
	driveCtl.setTurnPIDValues(1, 0, 0); // and here

	driveCtl.move(12); // should theoretically move 1 foot
	pros::delay(100);
	driveCtl.turnTo(90); // should turn 90 degrees

	// this DEFINITELY doesn't work anymore but might try anyway
	// driveCtl.triangleToPoint(10, 10); //should go 10 in left and 10 in forward
}

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
void opcontrol()
{
	std::cout << "Teleop start" << std::endl;
	// TEMPORARY! ENABLE FOR TESTING
	// autonomous();
	
	float y = sysConf::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
	float rot = sysConf::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

	while (true)
	{
		y = sysConf::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		rot = sysConf::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		/* code */
		sysConf::L1.move_voltage(y + rot);
		sysConf::L2.move_voltage(y + rot);
		sysConf::L3.move_voltage(y + rot);

		sysConf::R1.move_voltage(y - rot);
		sysConf::R2.move_voltage(y - rot);
		sysConf::R3.move_voltage(y - rot);
	}
	
	std::cout << "Program end" << std::endl;
}