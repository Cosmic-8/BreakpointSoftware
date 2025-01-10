#include "HAL/driveSys.hpp"
#include "util.hpp"
#ifndef ODOMETRY2_HPP
#define ODOMETRY2_HPP
    
    namespace softwareSubsystems {

        class odometryV2 {

        private:
            float diameter;

            double deltaL;
            double deltaR;
            double deltaB;

            double posL;
            double posR;
            double posB;

            double encoderPrevL;
            double encoderPrevR;
            double encoderPrevB;

            double encoderCurrentL;
            double encoderCurrentR;
            double encoderCurrentB;

            double deltaTheta;
            double prevTheta;
            double avgTheta;

            double leftFromCenter;
            double rightFromCenter;

            double deltaGlobalPosX;
            double deltaGlobalPosY;
            double globalTheta;

            double localDeltaX;
            double localDeltaY;

            double totalL;
            double totalR;
            double totalB;

            int countcycle;
            double globalPosX;
            double globalPosY;
            double currentTheta;

            //tells weather or not odometry has been setup
            bool isSetup = false;

            //private empty constructor
            odometryV2()
            {
            }

            //odometry object pointer
            static odometryV2* odometryV2_ptr;

        public:
            bool trackingInProgress;
            
            static odometryV2* getInstance();
            void setup(double startingX, double startingY, double startingHeading);
            double getXPos();
            double getYPos();
            double getRobotHeading();
            double getCurrentTheta();
            void trackPosition();
            void resetAll();
            void setPosition(double startingX, double startingY, double startingHeading);
        };

    }

    #endif