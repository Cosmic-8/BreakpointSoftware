#ifndef DRIVESYS_HPP
#define DRIVESYS_HPP
#include "config.hpp"


namespace HAL
{
    class driveSystem
    {
        
        private:  

        float LeftDistance = 0;
        float RightDistance = 0;
        
        public:

        //movement control
        void moveVoltage(float target);
        void moveVelocity(float target);
        void moveLeftVelocity(float target);
        void moveRightVelocity(float target);
        void moveLeftVoltage(float target);
        void moveRightVoltage(float target);
        void setBreakMode(pros::motor_brake_mode_e setting);
        void allStop();

        //telem
        void resetPosition();

        float getLeftDistance();
        float getRightDistance();
        float getLeftPosRaw();
        float getRightPosRaw();
    };
}

#endif