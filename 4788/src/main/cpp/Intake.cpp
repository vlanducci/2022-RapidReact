#include "Intake.h"
#include <iostream>

using namespace wml;
using namespace wml::controllers;

Intake::Intake(RobotMap::IntakeSystem &intakeSystem, Controllers &contGroup) : _intakeSystem(intakeSystem), _contGroup(contGroup) {
  intakeSystem.intakeSolenoid.SetTarget(wml::actuators::BinaryActuatorState::kReverse);
}

void Intake::teleopOnUpdate(double dt) {
  if (fabs(_contGroup.Get(ControlMap::Intake)) > 0.15) {
    _intakeSystem.indexWheelMotor.Set(_contGroup.Get(ControlMap::Intake));
    _intakeSystem.intakeMotor.Set(_contGroup.Get(ControlMap::Intake));
  }

  if (_contGroup.Get(ControlMap::IntakeSolenoid, wml::controllers::Controller::ButtonMode::ONRISE)) {
    toggle = !toggle;
  }
  if (toggle) {
    _intakeSystem.intakeSolenoid.SetTarget(wml::actuators::BinaryActuatorState::kForward);
  } else {
    // _intakeSystem.intakeSolenoid.SetTarget(wml::actuators::BinaryActuatorState::kReverse);
  }
}