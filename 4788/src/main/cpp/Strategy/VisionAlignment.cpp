#include <iostream>
#include <networktables/NetworkTableInstance.h>
#include "Strategy/VisionAlignment.h"
#include "ControlMap.h"

VisionAlignment::VisionAlignment(std::string name, Drivetrain &drivetrain, bool track) : wml::Strategy(name), _drivetrain(drivetrain), _drivetrainAngleStrategy("VisionAngle", drivetrain, _lastYaw), _track(track){
  Requires(&drivetrain);
  SetCanBeInterrupted(true);
}

void VisionAlignment::OnStart() {
  _drivetrainAngleStrategy.OnStart();
}

void VisionAlignment::OnUpdate(double dt) {
  double leftPower = 0, rightPower = 0;

  double xCords = _visionTable->GetEntry("targetPixelsX").GetDouble(0); 
  double yCords = _visionTable->GetEntry("targetPixelsY").GetDouble(0);
  double yawCords = _visionTable->GetEntry("targetYaw").GetDouble(0);
  double gyro = _drivetrain.GetConfig().gyro->GetAngle();
  double isFinished = _visionTable->GetEntry("Is finished").SetBoolean(_drivetrainAngleStrategy.IsFinished());

  if (std::abs(yawCords - _lastYaw) > 0.005)
    _drivetrainAngleStrategy.SetGoal(gyro + yawCords);

  _drivetrainAngleStrategy.OnUpdate(dt);

  if (!_track) {
    if (_drivetrainAngleStrategy.IsFinished())
      SetDone();
  }

  // nt::NetworkTableInstance::GetDefault().GetTable("testVisionTable")->GetEntry("lastYaw").SetDouble(_lastYaw);

  std::cout << "yawCord: " << yawCords << std::endl;
  std::cout << "gyro: " << gyro << std::endl;

  _lastYaw = yawCords;
}


// ------------- Speed for Shooter Thing -------------

VisionSnapStrat::VisionSnapStrat(std::string name) : wml::Strategy(name){
  SetCanBeInterrupted(true);
}

void VisionSnapStrat::OnStart() {}

void VisionSnapStrat::OnUpdate(double dt) {
  double pitchCords = _visionTable->GetEntry("targetPitch").GetDouble(0);

  std::cout << "pitch: " << pitchCords << std::endl;
}