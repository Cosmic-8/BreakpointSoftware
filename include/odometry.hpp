#include "api.h"
#include "config.hpp"
#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

    class odometry {

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

      public:
        bool trackingInProgress;
        odometry(double startingX, double startingY, double startingHeading);
        void setup(double startingX, double startingY, double startingHeading);
        double getXPos();
        double getYPos();
        double getRobotHeading();
        double getCurrentTheta();
        void trackPosition();
        void resetAll();
        void setPosition(double startingX, double startingY, double startingHeading);
    };

    #endif