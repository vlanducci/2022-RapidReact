#include "Robot.h"

using namespace frc;
using namespace wml;

// class Belevator {
//  private:
// 	wml::TalonSrx motor_1;
// 	wml::TalonSrx motor_2;
//  public:
// 	void new(wml::TalonSrx motor_1, wml::TalonSrx motor_2) {
		
// 	}
// }

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
				motorPower = xbox;
			}
		// Test if the robot is at the bottom
		} else if (_motor1->GetEncoderRotations() <= 0.1 || _motor2->GetEncoderTicks() <= 0.1) {
			if (xbox >= _deadzone) {
				motorPower = xbox;
			}
		// If neither of the limts are reached
		} else if (fabs(xbox) >= _deadzone) {
			motorPower = xbox;
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

wml::controllers::XboxController xbox = wml::controllers::XboxController{ 3 };
Belevator belev(0, 1, 0.5, 5, 0.3);

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {}

void Robot::DisabledInit() {
	InterruptAll(true);
}
void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {
	
}

// Manual Robot Logic
void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
	// Create variable for holding power and get the controller
	belev.set(xbox.GetAxis(xbox.kLeftYAxis));
} 

void Robot::TestInit() {}
void Robot::TestPeriodic() {}