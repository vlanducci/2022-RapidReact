#pragma once 

// General
#include <string>
#include <stdint.h>


// FRC/WPI
#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/DoubleSolenoid.h>
#include <frc/GenericHID.h>

#include <cameraserver/CameraServer.h>
#include <frc/DriverStation.h>

#include <frc/SpeedControllerGroup.h>
#include <frc/PowerDistribution.h>
#include <frc/Servo.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogInput.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/SmallString.h>

// WML
#include <WMLCtre.h>
#include <controllers/Controllers.h>
#include <actuators/BinaryServo.h>
#include <actuators/Compressor.h>
#include <NTProvider.h>
#include <actuators/DoubleSolenoid.h>
#include <actuators/VoltageController.h>
#include <Drivetrain.h>
#include <sensors/Encoder.h>
#include <sensors/LimitSwitch.h>
#include <sensors/NavX.h>
#include <sensors/PressureSensor.h>
#include <control/PIDController.h>
#include <MotionProfiling.h>
#include <Toggle.h>

#include <devices/StateDevice.h>
#include <strategy/StrategyController.h>
#include <strategy/MPStrategy.h>
#include <control/MotorFilters.h>
#include <Gearbox.h>
#include <strategy/Strategy.h>
#include <sensors/BinarySensor.h>

// WML Rev
#include <WMLRev.h>

// Local Files
#include "ControlMap.h"

struct RobotMap {
  // Controllers
  wml::controllers::XboxController xbox1{ ControlMap::Xbox1Port };
  wml::controllers::XboxController xbox2{ ControlMap::Xbox2Port };
  wml::controllers::SmartControllerGroup contGroup{ xbox1, xbox2};

  struct ControlSystem {
    wml::sensors::PressureSensor pressureSensor{ ControlMap::PressureSensorPort };
    wml::actuators::Compressor compressor{ ControlMap::CompressorPort, wml::actuators::PneumaticsModuleType::kCTRE, "Cj" };
  }; ControlSystem controlSystem;

  struct ExampleElevatorSystem {
    wml::TalonSrx elevatorMotor{ ControlMap::ElevatorMotorPort, 2048 };
    wml::actuators::DoubleSolenoid elevatorSolenoid{ ControlMap::PCModule, ControlMap::ElevatorSolenoidPort, 0.1 };
  }; ExampleElevatorSystem exampleElevatorSystem;

  struct IntakeSystem {
    wml::TalonSrx intakeMotor{ ControlMap::IntakeMotorPort, 2048 };
    wml::TalonSrx indexWheelMotor{ ControlMap::IndexWheelMotorPort, 2048 };
    wml::actuators::DoubleSolenoid intakeSolenoid{ ControlMap::PCModule, ControlMap::IntakeSolenoidPort, 0.1 };
  }; IntakeSystem intakeSystem;
};