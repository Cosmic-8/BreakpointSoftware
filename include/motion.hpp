#include "softwareSubsystems/odometry2.hpp"
#include "HAL/driveSys.hpp"
#include "api.h"
#include "util.hpp"
#ifndef DRIVETRAINCONTROLS_HPP
#define DRIVETRAINCONTROLS_HPP

    class motionProfiling {

      private:
        std::vector<double> timestamps;
        std::vector<double> positions;
        std::vector<double> velocities;
        std::vector<double> accelerations;
        double acceleration;
        double accelerationDist;
        double timeToMaxVelocity;
        double maxVelocity;
        double dt;
        double timeAtMaxVelocity;
        double totalTime;
        double totalDist;
        double profileMaxVelocity;
        double timeFromMaxVelocity;
        double timeToStill;
        double decelerationDist;
        double constantVelocityDist;
        double timeForConstantVelocity;
        double accelerationTime;
        double cruisingTime;
        double cruisingDist;
        double decelerationTime;

      public:
        void linearTrapezoidal(double maxAccel, double maxVel, double target);
        void sCurve1DOF(double target);
    };

    class drivePID {
      private:
        double kP;
        double kI;
        double kD;
        double totalError;
        double error;
        double derivative;
        double prevError;
        double position;
        double headingChange;
        double voltage;
        double moveTarget; //* rotations
        double turnTarget;
        double turnkP;
        double turnkI;
        double turnkD;
        double turnTotalError;
        double turnError;
        double turnDerivative;
        double turnPrevError;
        double driftTarget;
        double driftkP;
        double driftkI;
        double driftkD;
        double driftError;
        double driftTotalError;
        double driftDerivative;
        double driftPrevError;
        bool helperFunctions;

      public:
        drivePID();
        bool enableDriftCorrection;
        void setDriftCorrectionValues(double kP, double kI, double kD);
        double getDriftCorrection(double target);
        void resetDriftCorrection();
        void setPIDValues(double kP, double kI, double kD);
        void resetPID();
        void move(double target);
        void setTurnPIDValues(double kP, double kI, double kD);
        void resetTurnPID();
        void turnTo(double target); //* target is global value
        void triangleToPoint(double targetX, double targetY);
    };
#endif
