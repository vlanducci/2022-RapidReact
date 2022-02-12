#pragma once

#include <iostream>
#include "controllers/Controller.h"
#include "RobotMap.h"

class Auto {
public:
  Auto(Drivetrain &drivetrain) : _drivetrain(drivetrain) {}
  void init();
  void periodic(double dt);

private:
  // RobotMap::DrivebaseSystem &_drivebaseSystem;
  Drivetrain &_drivetrain;
};
