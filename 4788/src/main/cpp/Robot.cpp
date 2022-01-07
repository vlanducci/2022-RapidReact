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

wml::TalonSrx motor1 = wml::TalonSrx{ 1 };
wml::TalonSrx motor2 = wml::TalonSrx{ 2 };
const double kMaxSpeed = 0.5;
const double kLimit = 5;
double deadzone = 0.3;

wml::controllers::XboxController xbox = wml::controllers::XboxController{ 3 };

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
	double motorPower = 0;
	double controller = xbox.GetAxis(xbox.kLeftYAxis);

	// Test if the motor is at the limit
	if (motor1.GetEncoderRotations() >= kLimit || motor2.GetEncoderRotations() >= kLimit) {
		if (controller <= -deadzone) {
			motorPower = std::clamp(controller, -kMaxSpeed, 0.0);
		}
	// Test if the robot is at the bottom
	} else if (motor1.GetEncoderRotations() <= 0.1 || motor2.GetEncoderRotations() <= 0.1) {
		if (controller >= deadzone) {
			motorPower = std::clamp(controller, 0.0, kMaxSpeed);
		}
	// If neither of the limts are reached
	} else if (fabs(controller) >= deadzone) {
		motorPower = std::clamp(controller, -kMaxSpeed, kMaxSpeed);
	}

	motor1.Set(motorPower);
	motor2.Set(motorPower);
} 

void Robot::TestInit() {}
void Robot::TestPeriodic() {}