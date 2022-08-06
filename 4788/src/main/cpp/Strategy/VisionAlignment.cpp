#include <iostream>
#include <networktables/NetworkTableInstance.h>
#include "Strategy/VisionAlignment.h"
#include "ControlMap.h"
#include "cmath"

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


SnapStrat::SnapStrat(std::string name) : wml::Strategy(name){
  SetCanBeInterrupted(true);
}

void SnapStrat::OnUpdate(double dt) {
  double cameraAngle = 70;
  double pitch = 0;
  double tapeHeight = 0;
  double ringHeight = 0;
  double distanceToRing = 0.679+tapeHeight/tan(pitch*3.1416/180);  // Centre of ring in meters
  double exitVelocity = pow(((-4.9*pow(distanceToRing, 2)) / (pow(cos(70*3.1416/180), 2))) / (ringHeight-tan(70*3.1416/180)*distanceToRing), 0.5);
  double offSetFactor = 1.1; // of set function maybe linear?? Might be wrong
  double exitAngularVelocity = ((exitVelocity/((3.1416*4*25.4)/1000)) * (2*3.1416)) * offSetFactor;  // radians per second (times exitAngularVelocity by of set value)
}
