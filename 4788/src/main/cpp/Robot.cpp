#include "Robot.h"
#include "WMLCtre.h"

using namespace frc;
// using namespace wml;

// leftF m = ?
// leftB m = ?
// rightF m = ?
// rightB m = ?

// wml::TalonSrx {port}
// wml::controllers::Joystick {port}

// wml::TalonSrx leftF{99};
// wml::TalonSrx leftB{99};
// wml::TalonSrx rightF{99};
// wml::TalonSrx rightB{99};

wml::TalonSrx intakeM1{7};
wml::TalonSrx intakeM2{2};

wml::controllers::Joystick joy{0};
wml::controllers::XboxController xbox{ 1 };

class Drivetrain {
 public:
	Drivetrain(int topLeft = 0, int topRight = 1, int bottomLeft = 2, int bottomRight = 3) {
		// Right motor init
		_rightM1 = new wml::TalonSrx(topRight);
		_rightM2 = new wml::TalonSrx(bottomRight);

		// Left motor init
		_leftM1 = new wml::TalonSrx(topLeft);
		_leftM2 = new wml::TalonSrx(bottomLeft);

		_leftM1->SetInverted(false);
		_leftM2->SetInverted(true);
	}

	std::pair<double, double> get() {
		return {_left, _right};
	}

	/**
	 * @brief Set the Turret object
	 * 
	 * @param leftPower 
	 * @param rightPower 
	 * @return * Set 
	 */
	void setTurret(double leftPower, double rightPower) {
		_left = 0;
		_right = 0;

		// Left motors
		_leftM1->Set(leftPower);
		_leftM2->Set(leftPower);

		// Right motors
		_rightM1->Set(rightPower);
		_rightM2->Set(rightPower);
	}

	void set(double joyX, double joyY, double joyZ) {
		double left1 = 0, left2 = 0, right1 = 0, right2 = 0;

		// Forwards back
		if (fabs(joyY) > 0.15) {
			left1 += joyY;
			left2 += joyY;
			right1 += joyY;
			right2 += joyY;
		}

		// Left right
		if (fabs(joyX) > 0.15) {
			left1 += joyX;
			left2 += -joyX;
			right1 += -joyX;
			right2 += joyX;
		}

		// Rotation
		if (fabs(joyZ) > 0.15) {
			left1 += joyZ;
			left2 += joyZ;
			right1 += -joyZ;
			right2 += -joyZ;
		}
		_leftM1->Set(left1);
		_leftM2->Set(left2);
		_rightM1->Set(right1);
		_rightM2->Set(right2);	
	}

 private:
	wml::TalonSrx *_rightM1, *_rightM2, *_leftM1, *_leftM2;
	double _left = 0, _right = 0;
};

Drivetrain drive(14, 1, 6, 8);

const int deadzone = 0.15;
bool pressed = false;


class Belevator {
 public:
	Belevator(int motor1 = 5, int motor2 = 6, double maxSpeed = 0.5, double limit = 5, double deadzone = 0.3) {
		_motor1 = new wml::TalonSrx(motor1);
		_motor2 = new wml::TalonSrx(motor2);

		_maxSpeed = maxSpeed;
		_limit = limit;
		_deadzone = deadzone;
	}

	void set(double xbox) {
		double motorPower = 0;

		// //if (_motor1->GetEncoderRotations() >= _limit) {
		// 	if (xbox <= -_deadzone) {
		// 		motorPower = std::clamp(xbox, -_maxSpeed, 0.0);
		// 	}
		// // Test if the robot is at the bottom
		// //} else if (_motor1->GetEncoderRotations() <= 0.1) {
		// 	if (xbox >= _deadzone) {
		// 		motorPower = std::clamp(xbox, 0.0, _maxSpeed);
		//	}
		// If neither of the limts are reached
		//} else if (fabs(xbox) >= _deadzone) {
		motorPower = std::clamp(xbox, -_maxSpeed, _maxSpeed);
		//

		_motor1->Set(motorPower);
		_motor2->Set(motorPower);
	}
 private:
	wml::TalonSrx *_motor1, *_motor2;
	double _maxSpeed;
	double _limit;
	double _deadzone;
};

Belevator belev{3, 5, 0.5, 1, 0.2};

// Intake Wheel Motors
wml::TalonSrx wheelMotorL{2};
wml::TalonSrx wheelMotorR{7};

// Solenoids
wml::actuators::DoubleSolenoid pistonL{0, 1, 0.2}; // 0 = Forward, 1 = Reverse
wml::actuators::DoubleSolenoid pistonR{2, 3, 0.2}; // 2 = Forward, 3 = Reverse

// Compressor
wml::actuators::Compressor compressor{9};

// Xbox Controller 
// wml::controllers::XboxController xbox{1};

void Robot::RobotInit() {
	wheelMotorL.SetInverted(false);
	wheelMotorR.SetInverted(false);

}

void Robot::RobotPeriodic() {
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
	compressor.SetTarget(wml::actuators::kForward);
}

void Robot::TeleopPeriodic() {
	// double wheelSpeed = 0;

	// if(xbox.GetButton(xbox.kX)) {
	// 	pressed = true;
	// }

	// // Intake Wheels
	// if(xbox.GetAxis(xbox.kLeftThrottle) >= deadzone) {
	// 	wheelSpeed += xbox.GetAxis(xbox.kRightThrottle);
	// }

	// // Outake Wheels
	// if(xbox.GetAxis(xbox.kRightThrottle) >= deadzone) {
	// 	wheelSpeed += -xbox.GetAxis(xbox.kRightThrottle);
	// }

	// // Intake Pistons
	// if(pressed == true) {
	// 	pistonL.SetTarget(wml::actuators::kForward);
	// 	pistonR.SetTarget(wml::actuators::kForward);
	// } 
	
	// if(pressed == false) {
	// 	pistonL.SetTarget(wml::actuators::kReverse);
	// 	pistonR.SetTarget(wml::actuators::kReverse);
	// }

	// std::cout << "Robot code running" << std::endl;
	double intakeMPower = 0;

	// motors for intake
	// intakeMPower = 0.5;
	if (xbox.GetAxis(xbox.kRightThrottle) >= 0.15) {
		intakeMPower = xbox.GetAxis(xbox.kRightThrottle);
	}



	if (xbox.GetAxis(xbox.kLeftThrottle) >= 0.15) {
		intakeMPower = -xbox.GetAxis(xbox.kLeftThrottle);
	}

	// std::cout << "Intake power: " << intakeMPower << std::endl;
	
	// wheelMotorL.Set(wheelSpeed);
	// wheelMotorR.Set(wheelSpeed);


	intakeM1.Set(intakeMPower);
	intakeM2.Set(-intakeMPower);

	drive.set(joy.GetAxis(joy.kXAxis), -joy.GetAxis(joy.kYAxis), joy.GetAxis(joy.kZAxis));
	belev.set(-xbox.GetAxis(xbox.kLeftYAxis));
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}