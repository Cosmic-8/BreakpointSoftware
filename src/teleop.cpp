#include "teleop.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstddef>
#include <exception>
#include <iostream>
#include <ostream>

bool TeleoperationSystem::initialized = false;

TeleoperationSystem::TeleoperationSystem(pros::Motor fr, pros::Motor fl, pros::Motor rr, pros::Motor rl, pros::Controller controller) 
{
  std::cout << "Attempting to initialize Teleoperation system" << std::endl;
  if (initialized) 
  {
    std::cout << "Teleoperation System Error: Teleoperation system already initialized. Please shutdown the old instance before starting a new one." << std::endl;
    throw std::exception();
  }

  std::cout << "Created teleop system at " << this << std::endl;

  // copy motor referances
  this->m_fl = &fl;
  this->m_fr = &fr;
  this->m_rr = &rr;
  this->m_rl = &rl;
  this->m_il = nullptr;
  this->m_ir = nullptr;

  this->mainController = &controller;
  this->deadZone = 0.1;
}

TeleoperationSystem::TeleoperationSystem(pros::Motor fr, pros::Motor fl, pros::Motor rr, pros::Motor rl, pros::Controller controller, float deadZone)
{
  std::cout << "Attempting to initialize Teleoperation system" << std::endl;
  if (initialized) 
  {
    std::cout << "Teleoperation System Error: Teleoperation system already initialized. Please shutdown the old instance before starting a new one." << std::endl;
    throw std::exception();
  }

  std::cout << "Created teleop system at " << this << std::endl;

    // copy motor referances
    this->m_fl = &fl;
    this->m_fr = &fr;
    this->m_rr = &rr;
    this->m_rl = &rl;
    this->m_il = nullptr;
    this->m_ir = nullptr;

    this->mainController = &controller;
    this->deadZone = deadZone;
}

TeleoperationSystem::TeleoperationSystem(pros::Motor fr, pros::Motor fl, pros::Motor rr, pros::Motor rl, pros::Motor il, pros::Motor ir, pros::Controller controller, float deadZone)
{
  std::cout << "Attempting to initialize Teleoperation system" << std::endl;
  if (initialized) 
  {
    std::cout << "Teleoperation System Error: Teleoperation system already initialized. Please shutdown the old instance before starting a new one." << std::endl;
    throw std::exception();
  }

  std::cout << "Created teleop system at " << this << std::endl;

    // copy motor referances
    this->m_fl = &fl;
    this->m_fr = &fr;
    this->m_rr = &rr;
    this->m_rl = &rl;
    this->m_il = &il;
    this->m_ir = &ir;

    this->mainController = &controller;
    this->deadZone = deadZone;
}

TeleoperationSystem::~TeleoperationSystem() 
{ 
    std::cout << "Deconstructor called. Uninitializing teleop System and erasing from memory" << std::endl;
    initialized = false; 
}

void TeleoperationSystem::Start() 
{ 
    enabled = true;
    std::cout << "Teleop enabled" << std::endl;
}

void TeleoperationSystem::Stop() 
{ 
    enabled = false; 
    std::cout << "Teleop disabled" << std::endl;
}

void TeleoperationSystem::Update() {
  // this is a basic non-vector based holonomic drive system. A more complex
  // control scheme can be implemented in the future.
  //std::cout << "Teleop Enabled stat: " << enabled << std::endl;
  if (enabled) {
    float x = this->mainController->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    float y = -this->mainController->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    float rot = this->mainController->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // #ifdef DEBUG
    // if(cycle % 100 == 0)
    //   std::cout << "Updating motor commands. Inputs: " << x << " | " << y << " | " << rot << std::endl;

    // cycle++;
    // #endif

    //if (fabs(x) > deadZone || fabs(y) > deadZone || fabs(rot) > deadZone)
    //{
      #ifdef DEBUG
      if(cycle % 100 == 0)
        std::cout << "Updating motor commands. Inputs: " << x << " | " << y << " | " << rot << std::endl;

      cycle++;
      #endif
      m_fl->move_voltage((y + x + rot)*100);
      m_fr->move_voltage((-y + x + rot)*100);
      m_rl->move_voltage((y - x + rot)*100);
      m_rr->move_voltage((-y - x + rot)*100);
    //}
   // else 
    //{
      // #ifdef DEBUG
      // if(cycle % 100 == 0)
      // std::cout << "Updating motor commands. Ignored (deadzone) " << std::endl;
      // #endif
    //}
  }
}

void TeleoperationSystem::Update(bool debug) {
  // this is a basic non-vector based holonomic drive system. A more complex
  // control scheme can be implemented in the future.
  if (enabled) {
    float x = this->mainController->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    float y = -this->mainController->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    float rot = this->mainController->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    if (fabs(x) > deadZone || fabs(y) > deadZone || fabs(rot) > deadZone)
    {
        m_fl->move_voltage(y + x + rot);
        m_fr->move_voltage(-y + x + rot);
        m_rl->move_voltage(y - x + rot);
        m_rr->move_voltage(-y - x + rot);

        if(debug)
            std::cout << "x: " << x << " y: " << y << " rotation: " << rot << std::endl;
    }
  }
}