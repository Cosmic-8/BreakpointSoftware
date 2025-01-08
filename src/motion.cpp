#include "motion.hpp"
#include "pros/motors.h"

bool keepPositionTask = false;

    void motionProfiling::linearTrapezoidal(double maxAccel, double maxVel, double target) {
        HAL::driveSystem localDriveSys;
        //* https://github.com/MittyRobotics/motion-profile-generator/blob/master/src/main/java/com/amhsrobotics/motionprofile/TrapezoidalMotionProfile.java
        totalDist = target / (sysConf::wheelDiam * M_PI * sysConf::driveRatio); //* target in rotations
        accelerationTime = maxVel / maxAccel;
        decelerationTime = accelerationTime;

        accelerationDist = 0.5 * maxAccel * pow(accelerationTime, 2);
        decelerationDist = accelerationDist;

        cruisingDist = totalDist - accelerationDist * 2;
        cruisingTime = (cruisingDist / maxVel);
        maxVelocity = maxVel;

        if (accelerationDist > (totalDist / 2)) //! untested
        {
            accelerationTime = sqrt(totalDist / maxAccel);
            accelerationDist = 0.5 * maxAccel * pow(accelerationTime, 2);
            maxVelocity = maxAccel * accelerationTime;
        }

        cruisingDist = totalDist - 2 * accelerationDist;
        cruisingTime = cruisingDist / maxVelocity;

        decelerationTime = accelerationTime + cruisingTime;

        totalTime = accelerationTime * 2 + cruisingTime;
        totalDist = cruisingDist + accelerationDist * 2;

        double distTravelled = 0;
        double currentTime = 0;
        double startingTime = pros::millis();

        double motionprofilevel;

        while (currentTime <= totalTime) {
            currentTime = util::millisecondsToMinutes(pros::millis() - startingTime);

            if (currentTime < accelerationTime) {
                motionprofilevel = (maxAccel * currentTime);
                printf("Acceleration: %f\n", motionprofilevel);
                localDriveSys.moveVelocity(motionprofilevel);
            } else if (currentTime < decelerationTime) {
                motionprofilevel = maxVelocity;
                printf("Cruising: %f\n", motionprofilevel);
                localDriveSys.moveVelocity(maxVelocity);
            }

            else {
                motionprofilevel = maxVelocity - (maxAccel * (currentTime - decelerationTime));
                printf("Deceleration: %f\n", motionprofilevel);
                localDriveSys.moveVelocity(motionprofilevel);
            }
        }
        localDriveSys.allStop();

        //* ---------- pseudocode --------------------------------------------
        //* original pseudocode from ctrl alt ftc
        //* we first generate the motion profile

        //* this code should be good
        // accelerationTime = maxVel / maxAccel
        // accelerationDist = 0.5 * maxAccel * pow(accelerationTime, 2);

        //* get some value "distance" that is the total distance between target point and current point
        //* robot should already be angled towards the target point
        //* totalDist needs to be set to that distance value:
        // if (accelerationDist > (totalDist / 2)) //* if acceleration distance is greater than half total distance
        //     accelerationTime = sqrt((totalDist / 2) / (0.5 * maxAccel));

        // decelerationTime = accelerationTime
        // constantVelocityDist = totalDist - (2 * accelerationDist);
        // timeForConstantVelocity = constantVelocityDist / maxVelocity;
        // [constantVelStart] = accelerationTime;
        // [constantVelEnd] = accelerationTime + timeForConstantVelocity;
        // [decelerationStart] = [constantVelEnd]
        // [decelerationEnd] = [constantVelEnd + decelerationTime]
        // [totalProfileTime] = accelerationTime + timeForConstantVelocity + decelerationTime;
        //* some of the variables above can be combined

        //* then we apply the velocity based on where in the profile the robot is:
        // while(true){
        //     errorDist = totalDist - [the distance we've travelled since the start of the motion profile, in inches]
        //     [currentTime] = [time since the start of motion profile. Miliseconds if the rest of the motion profile is milliseconds]
        //
        //*    see where the robot is:
        //     if (we went over the motion profile){
        //         [not sure]
        //     }
        //     else if (currentTime < accelerationTime) { //* if we're accelerating
        //         drive->setLeftVelocity(0.5 * maxAccel * pow(currentTime, 2)) //* I think
        //         drive->setRightVelocity([the same]);
        //     }

        //     else if (currentTime < decelerationStart){ //* "if [accelerationEnd] < currentTime < constantVelEnd"
        //         accelerationDist = 0.5 * maxAccel * pow(accelerationTime, 2);
        //         drive->setLeftVelocity(accelerationDist + (maxVelocity * (currentTime - accelerationTime)))
        //         drive->setRightVelocity([the same]);
        //     }

        //     else { //* if none of the above, we must be decelerating
        //         accelerationDist = 0.5 * maxAccel * pow(accelerationTime, 2);
        //         constVelocityDistance = maxVelocity * timeForConstantVelocity;
        //         drive->setLeftVelocity(accelerationDist + constantVelocityDistance + maxVelocity * decelerationTime - 0.5 * maxAccel * pow(decelerationTime, 2))) // todo: check equation
        //         drive->setRightVelocity([the same]);
        //     }

        //}
        //* --------------------------------------------------------------
    }

    drivePID::drivePID() {
        resetDriftCorrection();
        resetPID();
        resetTurnPID();
        helperFunctions = false;
        setTurnPIDValues(1, 0, 0);
        setPIDValues(1, 0, 0);
        setDriftCorrectionValues(1, 0, 0);
    }

    void drivePID::setDriftCorrectionValues(double kP, double kI, double kD) {
        this->driftkP = kP;
        this->driftkI = kI;
        this->driftkD = kD;
    }

    double drivePID::getDriftCorrection(double target) {
        softwareSubsystems::odometryV2* odomV2 = softwareSubsystems::odometryV2::getInstance();
        driftTarget = target;
        if (enableDriftCorrection) {
            driftError = driftTarget - odomV2->getRobotHeading(); //should this be a plus? (it probably doesn't matter)
            driftTotalError += driftError;
            driftDerivative = driftError - driftPrevError;

            if (driftError <= 0.2) {
                driftError = 0;
            }

            return (error * kP) + (totalError * kI) + (derivative * kD);

            driftPrevError = driftError;
        } else {
            return 0.0;
        }
    }

    void drivePID::resetDriftCorrection() {
        driftTotalError = 0;
        driftError = 0;
        driftPrevError = 0;
        driftDerivative = 0;
    }

    void drivePID::setPIDValues(double kP, double kI, double kD) {
        // printf("Setting robot PID values");
        std::cout << "Setting robot PID values\n";
        this->kP = kP;
        this->kI = kI;
        this->kD = kD;
    }

    void drivePID::resetPID() {
        this->error = 0;
        this->prevError = 0;
        this->totalError = 0;
        this->derivative = 0;
    }

    void drivePID::move(double target) //* target in INCHES
    {
        //get subsystem instances
        softwareSubsystems::odometryV2* odomV2 = softwareSubsystems::odometryV2::getInstance();
        HAL::driveSystem localDriveSys;

        moveTarget = (target / (sysConf::wheelDiam * M_PI * sysConf::driveRatio));
        position = (localDriveSys.getRightPosRaw() + localDriveSys.getLeftPosRaw()) / 2;
        error = moveTarget - position;
        double directionTarget = odomV2->getRobotHeading();
        while (fabs(error > 0.1)) {
            position = (localDriveSys.getRightPosRaw() + localDriveSys.getLeftPosRaw()) / 2;
            error = moveTarget - position;
            totalError += error;
            derivative = error - prevError;
            voltage = (error * kP) + (totalError * kI) + (derivative * kD);
            headingChange = getDriftCorrection(directionTarget);
            if (headingChange >= 0) {
                if (voltage <= 127) {
                    localDriveSys.moveLeftVoltage((voltage - fabs(headingChange)) * 127);
                    localDriveSys.moveRightVoltage(voltage * 127);
                } else {
                    localDriveSys.moveLeftVoltage((127 - fabs(headingChange)) * 127);
                    localDriveSys.moveRightVoltage(voltage * 127);
                }
            } else {
                if (voltage <= 127) {
                    localDriveSys.moveLeftVoltage(voltage * 127);
                    localDriveSys.moveRightVoltage((voltage - fabs(headingChange)) * 127);
                } else {
                    localDriveSys.moveLeftVoltage(127);
                    localDriveSys.moveRightVoltage((127 - fabs(headingChange)) * 127);
                }
            }
            pros::delay(10);
        }
        localDriveSys.allStop();
        if (_generalDebug && helperFunctions == false)
            printf("Moved %f inches\n", target);
    }

    void drivePID::setTurnPIDValues(double kP, double kI, double kD) {
        this->turnkP = kP;
        this->turnkI = kI;
        this->turnkD = kD;
    }

    void drivePID::resetTurnPID() {
        this->turnError = 0;
        this->turnPrevError = 0;
        this->turnTotalError = 0;
        this->turnDerivative = 0;
    }

    void drivePID::turnTo(double target) {

        //get subsystem instances
        softwareSubsystems::odometryV2* odomV2 = softwareSubsystems::odometryV2::getInstance();
        HAL::driveSystem localDriveSys;
    
        std::cout << "From PID wheel diameter: " << sysConf::wheelDiam << "\n" << std::endl;
        if (_PIDdebug)
            printf("Turning\n");
        turnTarget = target;
        turnError = turnTarget - odomV2->getRobotHeading();
        totalError = 0;
        prevError = 0;
        while (fabs(turnError) > 1) {
            turnError = turnTarget - odomV2->getRobotHeading();
            if (_PIDdebug)
                std::cout << "turnerror = " << turnTarget << " - " << odomV2->getRobotHeading() << "\n" << std::endl;
            turnError = util::wrapDegrees(turnError);
            turnTotalError += turnError;
            turnDerivative = turnError - turnPrevError;

            voltage = (turnError * turnkP) + (turnTotalError * turnkI) + (turnDerivative * turnkD);

            turnPrevError = turnError;
            std::cout << "Voltage:" << voltage;
            localDriveSys.moveLeftVoltage(voltage);
            localDriveSys.moveRightVoltage(-voltage);
            pros::delay(10);
        }
        localDriveSys.allStop();
        if (_generalDebug == true && helperFunctions == false)
            printf("Turned to %f degrees\n", target);
    }

    void drivePID::triangleToPoint(double targetX, double targetY) {
        softwareSubsystems::odometryV2* odomV2 = softwareSubsystems::odometryV2::getInstance();

        helperFunctions == true;
        double linearError = sqrt(pow(targetX - odomV2->getXPos(), 2) + pow(targetY - odomV2->getYPos(), 2));
        double targetAngle = atan2((targetX - odomV2->getXPos()), (targetY - odomV2->getYPos())) * (180 / M_PI);

        turnError = util::wrapDegrees(turnError);

        turnTo(targetAngle);
        linearError = sqrt(pow(targetX - odomV2->getXPos(), 2) + pow(targetY - odomV2->getYPos(), 2));
        move(linearError);

        helperFunctions = false;

        if (_generalDebug)
            printf("Moved to (%f, %f)\n", targetX, targetY);
    }