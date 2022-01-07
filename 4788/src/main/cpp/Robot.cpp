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

wml::TalonSrx intakeM1{99};
wml::TalonSrx intakeM2{99};


wml::controllers::Joystick joy{0};
wml::controllers::XboxController xbox{1};

class Drivetrain {
 public:
	Drivetrain(int topLeft = 0, int topRight = 1, int bottomLeft = 2, int bottomRight = 3) {
		// Right motor init
		_rightM1 = new wml::TalonSrx(topRight);
		_rightM2 = new wml::TalonSrx(bottomRight);

		// Left motor init
		_leftM1 = new wml::TalonSrx(topLeft);
		_leftM2 = new wml::TalonSrx(bottomLeft);

		_leftM1->SetInverted(true);
		_leftM2->SetInverted(true);
	}

	std::pair<double, double> get() {
		return {_left, _right};
	}

	/**
	 * Set left and right drivetrain
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
			left1 += -joyX;
			left2 += joyX;
			right1 += joyX;
			right2 += -joyX;
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

Drivetrain drive(99, 99, 99, 99);

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {}

void Robot::DisabledInit() {
	InterruptAll(true);
}
void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

// Manual Robot Logic
void Robot::TeleopInit() {
	// put invert stuff here
}
void Robot::TeleopPeriodic() {
	std::cout << "Robot code running" << std::endl;
	double intakeMPower = 0;

	// motors for intake
	if (fabs(xbox.GetButton(xbox.kRightThrottle)) >= 0.15) {
		intakeMPower = xbox.GetButton(xbox.kRightThrottle);
	}

	intakeM1.Set(intakeMPower);
	intakeM2.Set(intakeMPower);

	drive.set(fabs(joy.GetAxis(joy.kXAxis)), fabs(joy.GetAxis(joy.kYAxis)), fabs(joy.GetAxis(joy.kZAxis)));
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}