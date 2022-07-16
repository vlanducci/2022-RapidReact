#pragma once

#include <iostream>
#include "controllers/Controller.h"
#include "RobotMap.h"
#include "RobotControl.h"
#include "control/MotorFilters.h"
#include "DriveToDistanceStrategy.h"

class VisionAlignment : public wml::Strategy {
 public:
  VisionAlignment(std::string name, Drivetrain &drivetrain, double continue);

  void OnStart() override;
  void OnUpdate(double dt) override;

 private:
  Drivetrain &_drivetrain;
  std::shared_ptr<nt::NetworkTable> _visionTable = nt::NetworkTableInstance::GetDefault().GetTable("photonvision/visionCam");
  DrivetrainAngleStrategy _drivetrainAngleStrategy;
  double _lastYaw = 0;
  double _accSpeed = 0.2;
  double _continue = 0; //0 false, 1 true
};