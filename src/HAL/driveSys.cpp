#include "driveSys.hpp"

namespace HAL
{

    void driveSystem::moveVoltage(float target)
    {
        sysConf::L1.move_voltage(target);
        sysConf::L2.move_voltage(target);
        sysConf::L3.move_voltage(target);

        sysConf::R1.move_voltage(target);
        sysConf::R2.move_voltage(target);
        sysConf::R3.move_voltage(target);
    }

    void driveSystem::moveVelocity(float target)
    {
        sysConf::L1.move_velocity(target);
        sysConf::L2.move_velocity(target);
        sysConf::L3.move_velocity(target);

        sysConf::R1.move_velocity(target);
        sysConf::R2.move_velocity(target);
        sysConf::R3.move_velocity(target);
    }

    void driveSystem::moveLeftVoltage(float target)
    {
        sysConf::L1.move_voltage(target);
        sysConf::L2.move_voltage(target);
        sysConf::L3.move_voltage(target);

    }

    void driveSystem::moveLeftVelocity(float target)
    {
        sysConf::L1.move_velocity(target);
        sysConf::L2.move_velocity(target);
        sysConf::L3.move_velocity(target);
    }

    void driveSystem::moveRightVoltage(float target)
    {
        sysConf::R1.move_voltage(target);
        sysConf::R2.move_voltage(target);
        sysConf::R3.move_voltage(target);

    }

    void driveSystem::moveRightVelocity(float target)
    {
        sysConf::R1.move_velocity(target);
        sysConf::R2.move_velocity(target);
        sysConf::R3.move_velocity(target);
    }

    void driveSystem::allStop()
    {
        sysConf::L1.brake();
        sysConf::L2.brake();
        sysConf::L3.brake();

        sysConf::R1.brake();
        sysConf::R2.brake();
        sysConf::R3.brake();
    }

    void driveSystem::setBreakMode(pros::motor_brake_mode_e setting)
    {
        sysConf::L1.set_brake_mode(setting);
        sysConf::L2.set_brake_mode(setting);
        sysConf::L3.set_brake_mode(setting);
        
        sysConf::R1.set_brake_mode(setting);
        sysConf::R2.set_brake_mode(setting);
        sysConf::R3.set_brake_mode(setting);
    }

    //telemetry

    void driveSystem::resetPosition()
    {
        sysConf::L1.tare_position();
        sysConf::L2.tare_position();
        sysConf::L3.tare_position();
        
        sysConf::R1.tare_position();
        sysConf::R2.tare_position();
        sysConf::R3.tare_position();
    }

    float driveSystem::getLeftDistance()
    {
        return sysConf::L2.get_position() * 2 * M_PI * 2;
    }

    float driveSystem::getRightDistance()
    {
        return sysConf::R2.get_position() * 2 * M_PI * 2;
    }
    
    float driveSystem::getLeftPosRaw()
    {
        return sysConf::L2.get_position();
    }

    float driveSystem::getRightPosRaw()
    {
        return sysConf::R2.get_position();
    }
}