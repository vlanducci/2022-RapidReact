#pragma once

<<<<<<< HEAD
#include <iostream>
#include "controllers/Controllers.h"
#include "RobotMap.h"

using Controllers = wml::controllers::SmartControllerGroup;

class Intake {
 public:
  Intake(RobotMap::IntakeSystem &intakeSystem, Controllers &contGroup);

  void teleopOnUpdate(double dt);
  void autoOnUpdate(double dt);
  void testOnUpdate(double dt);

 private:
  bool toggle = false;

  RobotMap::IntakeSystem &_intakeSystem;
  Controllers &_contGroup;
=======
#include "RobotMap.h";
#include "controllers/Controllers.h";

using Controllers = wml::controllers::SmartControllerGroup;

enum class IntakeStates{
  STOWED = 0,
  DEPLOYED
};

class Intake {
 public:
  Intake(RobotMap::IntakeSystem &intakeSystem, Controllers &contGroup);
  void teleopOnUpdate (double dt);
  void autoOnUpdate (double dt);
  void testOnUpdate (double dt);
 private:
  RobotMap::IntakeSystem &_intakeSystem;
  Controllers &_contGroup;

  double power;
  IntakeStates _intakeState{IntakeStates::STOWED};
>>>>>>> 6a855f1c1ce3c66854d1fc990ed457b289d92cde
};