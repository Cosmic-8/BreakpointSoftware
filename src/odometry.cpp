// #include "odometry.hpp"
// #include "util.hpp"
// #include <iostream>
// #include <ostream>

// //+--------------+
// //|  DEPRECATED  |
// //+--------------+

// //* math from M_PIlons doc: https://wiki.purduesigbots.com/software/odometry
// //* unable to link original pdf
// //* https://www.youtube.com/watch?v=Av9ZMjS--gY

// // todo: needs more testing

// // odometryStruct odomStruct{
// //     nullptr, //* left tracker
// //     nullptr, //* right tracker
// //     nullptr, //* center/back tracker
// //     nullptr, //* inertial sensor
// //     0,       //* wheel diameter (inches)
// //     0,       //* left-right distance from left tracking wheel to center (inches)
// //     0,       //* left-right distance from right tracking wheel to center (inches)
// //     0,       //* forward-backward distance from back tracking wheel to center (inches)
// // };

// namespace MatrixFoundations {

//     odometry::odometry(double startingX, double startingY, double startingHeading)
//     {
//         setup(startingX, startingY, startingHeading);
//     }

//     double startingTheta;
//     //* temporary solution to duplicate objects
//     void odometry::setup(double startingX, double startingY, double startingHeading) {
//         // if (odomStruct.leftTracker == nullptr) {
//         //     std::cout << "Nullptr\n";
//         // }
//         hardwareSubsystems::trackingSystem* trackingSys = hardwareSubsystems::trackingSystem::getInstance();
//         std::cout << "odometry: WARNING: Deprecated class in use!" << std::endl;
//         this->resetAll();
//         startingTheta = util::degreesToRadians(startingHeading);
//         printf("Starting theta: %f", startingTheta);
//         if (trackingSys->getImuAvailable())
//             trackingSys->setHeading(startingHeading);
//         if (_odometryLogging)
//             printf("Left encoder centidegrees: %f, Right encoder centidegrees: %f, Back encoder centidegrees: %f, Diameter: %f, Left from center: %f, Right from center: %f, Back from center: %f\n", trackingSys->getLeftPosition(), trackingSys->getRightPosition(), trackingSys->getAuxPosition(), trackingSys->getDiameter(), trackingSys->getLeftOffset(), trackingSys->getRightOffset(), trackingSys->getAuxOffset());
//         // this->diameter = odomStruct.wheelDiameter;
//         // printf("Wheel diameter: %f\n", odomStruct.wheelDiameter);
//         this->globalPosX = startingX;
//         this->globalPosY = startingY;
//         this->globalTheta = startingTheta;
//         this->prevTheta = globalTheta;
//     }

//     void odometry::trackPosition() {
//         // pros::lcd::initialize();
//         pros::Task task{
//             [=] {
//                 hardwareSubsystems::trackingSystem* trackingSys = hardwareSubsystems::trackingSystem::getInstance();
//                 while (true) {
//                     encoderCurrentL = util::centidegreesToRotations(trackingSys->getLeftPosition());
//                     encoderCurrentR = util::centidegreesToRotations(trackingSys->getRightPosition());
//                     encoderCurrentB = util::centidegreesToRotations(trackingSys->getAuxPosition());
//                     if (_odometryLogging)
//                         printf("Encoder: %f, %f, %f\n", encoderCurrentL, encoderCurrentR, encoderCurrentB);

//                     deltaL = (encoderCurrentL - encoderPrevL) * trackingSys->getDiameter() * M_PI;
//                     deltaR = (encoderCurrentR - encoderPrevR) * trackingSys->getDiameter() * M_PI;
//                     deltaB = (encoderCurrentB - encoderPrevB) * trackingSys->getDiameter() * M_PI;

//                     // pros::lcd::print(2, "* %f * %f", odomStruct.wheelDiameter, M_PI);

//                     totalR += deltaR;
//                     totalL += deltaL;
//                     totalB += deltaB;

//                     currentTheta = (startingTheta + ((totalL - totalR) / (trackingSys->getLeftOffset() + trackingSys->getRightOffset())));
//                     // // pros::lcd::print(1, "%f - ((%f - %f)/(%f + %f))", startingTheta, totalL, totalR, odomStruct.leftFromCenter, odomStruct.rightFromCenter);

//                     currentTheta = util::wrapRadians(currentTheta);

//                     if (_showOdomMath) {
//                         printf("Currenttheta = %f + ((%f + %f) / (%f + %f)) = %f\n", startingTheta, totalL, totalR, trackingSys->getLeftOffset(), trackingSys->getRightOffset(), currentTheta);
//                     }

//                     deltaTheta = currentTheta - prevTheta;

//                     globalTheta += deltaTheta;

//                     if (deltaTheta == 0) //* if going straight forwards:
//                     {
//                         localDeltaX = deltaB;
//                         localDeltaY = deltaR;
//                     } else {
//                         localDeltaX = 2 * sin(deltaTheta / 2) * ((deltaB / deltaTheta) + trackingSys->getAuxOffset());
//                         localDeltaY = 2 * sin(deltaTheta / 2) * ((deltaR / deltaTheta) + trackingSys->getRightOffset());
//                         if (_showOdomMath) {
//                             printf("localDeltaY = 2 * sin(%f) * ((%f / %f) + 1.8125) = %f\n", globalTheta, deltaR, deltaTheta, localDeltaY);
//                         }
//                     }

//                     // pros::lcd::print(1, "Local delta x: %f, local delta y: %f", localDeltaX, localDeltaY);

//                     avgTheta = prevTheta + (deltaTheta / 2);

//                     //* calculate delta x and y:
//                     deltaGlobalPosX = (localDeltaX * cos(avgTheta)) - (localDeltaY * sin(avgTheta));
//                     deltaGlobalPosY = (localDeltaX * sin(avgTheta)) + (localDeltaY * cos(avgTheta));

//                     globalPosX += deltaGlobalPosX; // todo: make sure positive/negative is correct
//                     globalPosY += deltaGlobalPosY;

//                     prevTheta = currentTheta;
//                     encoderPrevL = encoderCurrentL;
//                     encoderPrevR = encoderCurrentR;
//                     encoderPrevB = encoderCurrentB;
//                     // // pros::lcd::print(0, "%f, %f, %f\n", globalPosX, globalPosY, radiansToDegrees(globalTheta));

//                     if (countcycle % 11 == 0 && _odometryLogging) {
//                         printf("Final values, %d: %f, %f, %f\n", countcycle, globalPosX, globalPosY, (globalTheta * 180 / M_PI));
//                     }

//                     countcycle++;
//                     pros::delay(10);
//                 }
//             }};
//     }

//     double odometry::getXPos() {
//         return globalPosX;
//     }

//     double odometry::getYPos() {
//         return globalPosY;
//     }

//     double odometry::getRobotHeading() {
//         return util::radiansToDegrees(globalTheta);
//     }

//     double odometry::getCurrentTheta() {
//         return globalTheta;
//     }

//     void odometry::setPosition(double startingX, double startingY, double startingHeading) {
//         hardwareSubsystems::trackingSystem* trackingSys = hardwareSubsystems::trackingSystem::getInstance();
//         this->globalPosX = startingX;
//         this->globalPosY = startingY;
//         this->currentTheta = util::degreesToRadians(startingHeading);
//         trackingSys->setHeading(startingHeading);
//     }

//     void odometry::resetAll() {
//         // todo: what exactly should I be resetting?
//         // if (odomStruct.leftTracker != nullptr)
//         //     odomStruct.leftTracker->reset_position();
//         // if (odomStruct.rightTracker != nullptr)
//         //     odomStruct.rightTracker->reset_position();
//         // if (odomStruct.centerTracker != nullptr)
//         //     odomStruct.centerTracker->reset_position();
//         hardwareSubsystems::trackingSystem* trackingSys = hardwareSubsystems::trackingSystem::getInstance();

//         trackingSys->resetAll(); //functionally equivalent

//         encoderPrevB = 0;
//         encoderPrevR = 0;
//         encoderPrevL = 0;
//         encoderCurrentB = 0;
//         encoderCurrentR = 0;
//         encoderCurrentL = 0;
//         posB = 0;
//         posR = 0;
//         posL = 0;
//         deltaGlobalPosX = 0;
//         deltaGlobalPosY = 0;
//         deltaB = 0;
//         deltaR = 0;
//         deltaL = 0;
//         deltaTheta = 0;
//         if (_odometryLogging) {
//             printf("Reset encoders\n");
//         }
//     }
// }