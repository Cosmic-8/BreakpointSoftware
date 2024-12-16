#ifndef TELEOP
#define TELEOP
#include "pros/misc.hpp"
#include "pros/motors.hpp"

//#define DEBUG //Uncomment for debug prints

//Header file for teleoparation system
//this system handles robot movement from user input.

class TeleoperationSystem
{
    public:
    
    TeleoperationSystem(pros::Motor fr, pros::Motor fl, pros::Motor rr, pros::Motor rl, pros::Controller controller); //basic constructor
    TeleoperationSystem(pros::Motor fr, pros::Motor fl, pros::Motor rr, pros::Motor rl, pros::Controller controller, float deadZone); //overload with deadzone
    TeleoperationSystem(pros::Motor fr, pros::Motor fl, pros::Motor rr, pros::Motor rl, pros::Motor il, pros::Motor ir, pros::Controller controller, float deadZone); //overload with intermediate motors
    ~TeleoperationSystem();

    void Start(); //makes controller start listening to update calls
    void Stop(); //makes controller disreguard update calls
    void Update(); //runs an update cycle that updates motor parameters
    void Update(bool debug); //runs an update cycle that updates motor parameters and also prints data to terminal
    
    private:
    static bool initialized; //keeps track of if an instance is already in existance
    int cycle; //keeps track of update cycle count
    bool enabled; //tells the internal task when to exit
    float deadZone; //controller stick deadzone

    //motors
    pros::Motor* m_fr;
    pros::Motor* m_fl;
    pros::Motor* m_rr;
    pros::Motor* m_rl;
    pros::Motor* m_il;
    pros::Motor* m_ir;
    pros::Controller* mainController;

    
};



#endif