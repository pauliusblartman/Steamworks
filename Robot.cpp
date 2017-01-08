#include "Robot.h"


	Robot::Robot() :
			robotDrive(Constants::frontLeftChannel, Constants::rearLeftChannel, Constants::frontRightChannel, Constants::rearRightChannel),
			driveStick(Constants::driveStickChannel),
      gyro(I2C::Port::kMXP, 200)
	{
		robotDrive.SetExpiration(0.1);
		robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, false);
    robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, false);
		robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, false);
    robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, false);
	}

	/**
	 * Runs the motors with Mecanum drive.
	 */
	void Robot::OperatorControl()
	{
		robotDrive.SetSafetyEnabled(false);
		while (IsOperatorControl() && IsEnabled())
		{
			robotDrive.MecanumDrive_Cartesian(stick.GetX(), stick.GetY(), stick.GetZ(), gyro.GetYaw());

			Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		}
	}

	void Robot::Autonomous() {

	}
START_ROBOT_CLASS(Robot)
