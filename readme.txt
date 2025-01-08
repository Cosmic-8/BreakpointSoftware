Welcome to the Breakpoint Software repo
Here you can find all versions of the robot software used by the Breakpoint team of 7th period

For info on the algoithems in use, see the following:
PID: https://wiki.purduesigbots.com/software/control-algorithms/pid-controller
Odometry: https://wiki.purduesigbots.com/software/odometry

Note that there are a few different features still present in the ported code that are no longer in use and/or don't work.

Class/File reference:
 _______________________________________________________________________________________________________________________________________________
| Files                        | Class          | Function                                                   | Notes                            |
|______________________________|________________|____________________________________________________________|__________________________________|
| Motion.hpp                   | drivePID       | Provides primary autonomous movement and control functions |                                  |
|______________________________|________________|____________________________________________________________|__________________________________|
| Config.hpp                   | N\A            | Provides system configs for devices and tuneing values     | This file is higly volatile.     |
|                              |                |                                                            | Missing declarations will result |
|                              |                |                                                            | in catastrophic failure          |
|______________________________|________________|____________________________________________________________|__________________________________|
| HAL/driveSys.hpp             | driveSysten    | Provides hardware abstraction for drivetrain functions     |                                  |
|______________________________|________________|____________________________________________________________|__________________________________|
| softwareSubsystems/odometry2 | odometryV2     | Provides facilites for position tracking                   | This is a singleton class        |
|______________________________|________________|____________________________________________________________|__________________________________|
List incomplete (WIP)
