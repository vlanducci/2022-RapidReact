#pragma once

#include "Calibration.h"
#include "Robot.h"
#include "Shooter.h"

enum class ShooterCalibrationState {
  kInit,
  kDriverDriveWait,
  kSpinUp,
  kFire,
  kDriverConfirm,
  kDone
};

struct RecordedValues {
  double visionArea;
  double visionPitch;
  double visionYaw;
  double speedSet;
  double speedActual;
  double batteryVoltage;
  bool didHit = false;
};

constexpr double MIN_SPEED = 10;
constexpr double MAX_SPEED = 50;

class ShooterCalibration : public wml::Strategy {
 public:
  ShooterCalibration(std::string name, Shooter &shooter);

  void OnStart() override;
  void OnUpdate(double dt) override;

 private:
  Shooter &_shooter;
  std::shared_ptr<nt::NetworkTable> _visionTable = nt::NetworkTableInstance::GetDefault().GetTable("photonvision/visionCam");
  ShooterCalibrationState _state{ ShooterCalibrationState::kDriverDriveWait };
  RecordedValues _values; 
  double _speed;
};

