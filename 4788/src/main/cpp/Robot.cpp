#include "Robot.h"

using namespace frc;
// using namespace wml;

const int deadzone = 0.15;
bool pressed = false;

wml::controllers::XboxController xbox{ 1 };

class Belevator {
 public:
	Belevator(int motor1 = 0, int motor2 = 1, double maxSpeed = 0.5, double limit = 5, double deadzone = 0.3) {
		_motor1 = new wml::TalonSrx(motor1);
		_motor2 = new wml::TalonSrx(motor2);

		_maxSpeed = maxSpeed;
		_limit = limit;
		_deadzone = deadzone;
	}

	void set(double xbox) {
		double motorPower = 0;

		if (_motor1->GetEncoderRotations() >= _limit || _motor2->GetEncoderTicks() >= _limit) {
			if (xbox <= -_deadzone) {
				motorPower = std::clamp(xbox, -_maxSpeed, 0.0);
			}
		// Test if the robot is at the bottom
		} else if (_motor1->GetEncoderRotations() <= 0.1 || _motor2->GetEncoderTicks() <= 0.1) {
			if (xbox >= _deadzone) {
				motorPower = std::clamp(xbox, 0.0, _maxSpeed);
			}
		// If neither of the limts are reached
		} else if (fabs(xbox) >= _deadzone) {
			motorPower = std::clamp(xbox, -_maxSpeed, _maxSpeed);
		}

		_motor1->Set(motorPower);
		_motor2->Set(motorPower);
	}
 private:
	wml::TalonSrx *_motor1, *_motor2;
	double _maxSpeed;
	double _limit;
	double _deadzone;
};

Belevator belev(5, 6, 0.5, 1, 0.3);

// Intake Wheel Motors
wml::TalonSrx wheelMotorL{7};
wml::TalonSrx wheelMotorR{8};

// Solenoids
wml::actuators::DoubleSolenoid pistonL{0, 1, 0.2}; // 0 = Forward, 1 = Reverse
wml::actuators::DoubleSolenoid pistonR{2, 3, 0.2}; // 2 = Forward, 3 = Reverse

// Compressor
wml::actuators::Compressor compressor{0};

// Xbox Controller
// wml::controllers::XboxController xbox{1};

void Robot::RobotInit() {
	wheelMotorL.SetInverted(false);
	wheelMotorR.SetInverted(false);
}

void Robot::RobotPeriodic() {
	compressor.SetTarget(wml::actuators::kForward);
}

void Robot::DisabledInit() {
	InterruptAll(true);
}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {
	
}

// Manual Robot Logic
void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
	double wheelSpeed = 0;

	if(xbox.GetButton(xbox.kX)) {
		pressed = true;
	}

	// Intake Wheels
	if(xbox.GetAxis(xbox.kLeftThrottle) >= deadzone) {
		wheelSpeed += xbox.GetAxis(xbox.kRightThrottle);
	}

	// Outake Wheels
	if(xbox.GetAxis(xbox.kRightThrottle) >= deadzone) {
		wheelSpeed += -xbox.GetAxis(xbox.kRightThrottle);
	}

	// Intake Pistons
	if(pressed == true) {
		pistonL.SetTarget(wml::actuators::kForward);
		pistonR.SetTarget(wml::actuators::kForward);
	} 
	
	if(pressed == false) {
		pistonL.SetTarget(wml::actuators::kReverse);
		pistonR.SetTarget(wml::actuators::kReverse);
	}

	wheelMotorL.Set(wheelSpeed);
	wheelMotorR.Set(wheelSpeed);

	belev.set(xbox.GetAxis(xbox.kLeftYAxis));
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}