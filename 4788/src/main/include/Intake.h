#pragma once

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
};