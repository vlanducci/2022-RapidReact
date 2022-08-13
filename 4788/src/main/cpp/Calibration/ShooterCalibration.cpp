#include <iostream>
#include <networktables/NetworkTableInstance.h>
#include "Calibration/ShooterCalibration.h"
#include "ControlMap.h"

ShooterCalibration::ShooterCalibration(std::string name, Shooter &shooter) : wml::Strategy(name), _shooter(shooter) {
  Requires(&shooter);
  SetCanBeInterrupted(true);
}

void ShooterCalibration::OnStart() {}

void ShooterCalibration::OnUpdate(double dt) {
  switch (_state) {
  case ShooterCalibrationState::kInit:
    break;

  case ShooterCalibrationState::kDriverDriveWait:
    if (wml::controllers::XboxController::kA) {
      _state == ShooterCalibrationState::kSpinUp;
    }
    break;

  case ShooterCalibrationState::kSpinUp:
    _shooter.setPID(_speed, dt);
    _state == ShooterCalibrationState::kFire;
    break;

  case ShooterCalibrationState::kFire:
    _shooter.calculatePID(_speed, dt);
    _state == ShooterCalibrationState::kDriverConfirm;
    break;

  case ShooterCalibrationState::kDriverConfirm:
    std::cout << "Save?" << std::endl;
    if (wml::controllers::XboxController::kA) {
      std::cout << "Save" << std::endl;
      _speed = _speed + ((MIN_SPEED + MAX_SPEED) / 10);
      if (_speed >= MAX_SPEED) {
        _state == ShooterCalibrationState::kSpinUp;
      } else {
        _state == ShooterCalibrationState::kDriverConfirm;
      }
    } else if (wml::controllers::XboxController::kB) {
      std::cout << "Dont Save" << std::endl;
      _speed = _speed + ((MIN_SPEED + MAX_SPEED) / 10);
      if (_speed >= MAX_SPEED) {
        _state == ShooterCalibrationState::kSpinUp;
      } else {
        _state == ShooterCalibrationState::kDriverConfirm;
      }
    }
    break;
  }
}

