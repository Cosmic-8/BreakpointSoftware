Welcome to the Breakpoint Software repo
Here you can find all versions of the robot software used by the Breakpoint team of 7th period

For info on the algoithems in use, see the following:
PID: https://wiki.purduesigbots.com/software/control-algorithms/pid-controller
Odometry: https://wiki.purduesigbots.com/software/odometry

Note that there are a few different features still present in the ported code that are no longer in use and/or don't work.

Class/File reference:
 __________________________________________________________________________________________________________________________
| Files                        | Class          | Function                              | Notes                            |
|______________________________|________________|_______________________________________|__________________________________|
| Motion.hpp                   | drivePID       | Provides primary autonomous movement  |                                  |
|                              |                | and control functions                 |                                  | 
|______________________________|________________|_______________________________________|__________________________________|
| Config.hpp                   | N\A            | Provides system configs for           | This file is higly volatile.     |
|                              |                | all devices and tuning values         | Missing declarations will result |
|                              |                |                                       | in catastrophic failure          |
|______________________________|________________|_______________________________________|__________________________________|
| HAL/driveSys.hpp             | driveSysten    | Provides hardware abstraction for     |                                  |
|                              |                | drivetrain functions                  |                                  |
|______________________________|________________|_______________________________________|__________________________________|
| softwareSubsystems/odometry2 | odometryV2     | Provides position tracking            | This is a singleton class        |
|______________________________|________________|_______________________________________|__________________________________|
List incomplete (WIP)

