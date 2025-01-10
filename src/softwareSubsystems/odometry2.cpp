#include "softwareSubsystems/odometry2.hpp"

softwareSubsystems::odometryV2* softwareSubsystems::odometryV2::odometryV2_ptr = nullptr;

//* math from M_PIlons doc: https://wiki.purduesigbots.com/software/odometry
//* unable to link original pdf
//* https://www.youtube.com/watch?v=Av9ZMjS--gY

// todo: needs more testing

// odometryStruct odomStruct{
//     nullptr, //* left tracker
//     nullptr, //* right tracker
//     nullptr, //* center/back tracker
//     nullptr, //* inertial sensor
//     0,       //* wheel diameter (inches)
//     0,       //* left-right distance from left tracking wheel to center (inches)
//     0,       //* left-right distance from right tracking wheel to center (inches)
//     0,       //* forward-backward distance from back tracking wheel to center (inches)
// };

    namespace softwareSubsystems
    {

        odometryV2* odometryV2::getInstance()
        {
            //lock context switcher
            pros::Mutex mutex;
            mutex.take();

            if(odometryV2_ptr == nullptr)
            {
                odometryV2_ptr = new odometryV2();
                std::cout << "New odometry object created with address: " << odometryV2_ptr << "NOTE: this address should not change" << std::endl;
                return odometryV2_ptr;
            }
            else 
            {
                std::cout << "Giving new process odom object with address: " << odometryV2_ptr << "NOTE: this address should not change" << std::endl;
                return odometryV2_ptr;
            }

            

            //unlock context switching
            mutex.give();
        }

        double startingTheta;
        //* temporary solution to duplicate objects
        void odometryV2::setup(double startingX, double startingY, double startingHeading) {
            // if (odomStruct.leftTracker == nullptr) {
            //     std::cout << "Nullptr\n";
            // }

            HAL::driveSystem localDriveSys;

            this->resetAll();
            startingTheta = util::degreesToRadians(startingHeading);
            printf("Starting theta: %f", startingTheta);
            
            sysConf::imu.set_heading(startingHeading);

            if (_odometryLogging)
                printf("Left encoder centidegrees: %f, Right encoder centidegrees: %f, Back encoder centidegrees: %f, Diameter: %f, Left from center: %f, Right from center: %f, Back from center: %f\n", localDriveSys.getLeftPosRaw(), localDriveSys.getRightPosRaw(), sysConf::rearEncoder.get_position(), sysConf::wheelDiam, sysConf::leftOffset, sysConf::rightOffset, sysConf::rearOffset);
            // this->diameter = odomStruct.wheelDiameter;
            // printf("Wheel diameter: %f\n", odomStruct.wheelDiameter);
            this->globalPosX = startingX;
            this->globalPosY = startingY;
            this->globalTheta = startingTheta;
            this->prevTheta = globalTheta;
        }

        void odometryV2::trackPosition() {
            // pros::lcd::initialize();
            
            pros::Task task{
                [=] {
                    HAL::driveSystem localDriveSys;
                    while (true) {
                        pros::Mutex mutex;
                        mutex.take();

                        encoderCurrentL = localDriveSys.getLeftPosRaw();
                        encoderCurrentR = localDriveSys.getRightPosRaw();
                        encoderCurrentB = util::centidegreesToRotations(sysConf::rearEncoder.get_position());
                        if (_odometryLogging)
                            printf("Encoder: %f, %f, %f\n", encoderCurrentL, encoderCurrentR, encoderCurrentB);

                        deltaL = (encoderCurrentL - encoderPrevL) * sysConf::wheelDiam * M_PI;
                        deltaR = (encoderCurrentR - encoderPrevR) * sysConf::wheelDiam * M_PI;
                        deltaB = (encoderCurrentB - encoderPrevB) * sysConf::rearEncoderWheelDiam * M_PI;

                        // pros::lcd::print(2, "* %f * %f", odomStruct.wheelDiameter, M_PI);

                        totalR += deltaR;
                        totalL += deltaL;
                        totalB += deltaB;

                        currentTheta = (startingTheta + ((totalL - totalR) / sysConf::leftOffset + sysConf::rightOffset));
                        // // pros::lcd::print(1, "%f - ((%f - %f)/(%f + %f))", startingTheta, totalL, totalR, odomStruct.leftFromCenter, odomStruct.rightFromCenter);

                        currentTheta = util::wrapRadians(currentTheta);

                        if (_showOdomMath) {
                            printf("Currenttheta = %f + ((%f + %f) / (%f + %f)) = %f\n", startingTheta, totalL, totalR, sysConf::leftOffset, sysConf::rightOffset, currentTheta);
                        }

                        deltaTheta = currentTheta - prevTheta;

                        globalTheta += deltaTheta;

                        if (deltaTheta == 0) //* if going straight forwards:
                        {
                            localDeltaX = deltaB;
                            localDeltaY = deltaR;
                        } else {
                            localDeltaX = 2 * sin(deltaTheta / 2) * ((deltaB / deltaTheta) + sysConf::rearOffset);
                            localDeltaY = 2 * sin(deltaTheta / 2) * ((deltaR / deltaTheta) + sysConf::rightOffset);
                            if (_showOdomMath) {
                                printf("localDeltaY = 2 * sin(%f) * ((%f / %f) + 1.8125) = %f\n", globalTheta, deltaR, deltaTheta, localDeltaY);
                            }
                        }

                        // pros::lcd::print(1, "Local delta x: %f, local delta y: %f", localDeltaX, localDeltaY);

                        avgTheta = prevTheta + (deltaTheta / 2);

                        //* calculate delta x and y:
                        deltaGlobalPosX = (localDeltaX * cos(avgTheta)) - (localDeltaY * sin(avgTheta));
                        deltaGlobalPosY = (localDeltaX * sin(avgTheta)) + (localDeltaY * cos(avgTheta));

                        globalPosX += deltaGlobalPosX; // todo: make sure positive/negative is correct
                        globalPosY += deltaGlobalPosY;

                        prevTheta = currentTheta;
                        encoderPrevL = encoderCurrentL;
                        encoderPrevR = encoderCurrentR;
                        encoderPrevB = encoderCurrentB;
                        // // pros::lcd::print(0, "%f, %f, %f\n", globalPosX, globalPosY, radiansToDegrees(globalTheta));

                        if (countcycle % 11 == 0 && _odometryLogging) {
                            printf("Final values, %d: %f, %f, %f\n", countcycle, globalPosX, globalPosY, (globalTheta * 180 / M_PI));
                        }

                        countcycle++;
                        mutex.give();
                        pros::delay(10);
                    }
                }};
        }

        double odometryV2::getXPos() {
            return globalPosX;
        }

        double odometryV2::getYPos() {
            return globalPosY;
        }

        double odometryV2::getRobotHeading() {
            return util::radiansToDegrees(globalTheta);
        }

        double odometryV2::getCurrentTheta() {
            return globalTheta;
        }

        void odometryV2::setPosition(double startingX, double startingY, double startingHeading) {
            this->globalPosX = startingX;
            this->globalPosY = startingY;
            this->currentTheta = util::degreesToRadians(startingHeading);
            sysConf::imu.set_heading(startingHeading);
        }

        void odometryV2::resetAll() {
            // todo: what exactly should I be resetting?
            // if (odomStruct.leftTracker != nullptr)
            //     odomStruct.leftTracker->reset_position();
            // if (odomStruct.rightTracker != nullptr)
            //     odomStruct.rightTracker->reset_position();
            // if (odomStruct.centerTracker != nullptr)
            //     odomStruct.centerTracker->reset_position();

            HAL::driveSystem localDriveSys;
            localDriveSys.resetPosition(); 

            encoderPrevB = 0;
            encoderPrevR = 0;
            encoderPrevL = 0;
            encoderCurrentB = 0;
            encoderCurrentR = 0;
            encoderCurrentL = 0;
            posB = 0;
            posR = 0;
            posL = 0;
            deltaGlobalPosX = 0;
            deltaGlobalPosY = 0;
            deltaB = 0;
            deltaR = 0;
            deltaL = 0;
            deltaTheta = 0;
            if (_odometryLogging) {
                printf("Reset encoders\n");
            }
        }

    }