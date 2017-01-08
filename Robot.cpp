#include "Robot.h"

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
    // Channels for the wheels




	Robot::Robot() :
			robotDrive(Constants::frontLeftChannel, Constants::rearLeftChannel,
					Constants::frontRightChannel, Constants::rearRightChannel),	// these must be initialized in the same order
			stick(Constants::driveStickChannel)								// as they are declared above.
	{
		robotDrive.SetExpiration(0.1);
		robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);	// invert the left side motors
		robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);	// you may need to change or remove this to match your robot
	}

	/**
	 * Runs the motors with Mecanum drive.
	 */
	void Robot::OperatorControl()
	{
		robotDrive.SetSafetyEnabled(false);
		while (IsOperatorControl() && IsEnabled())
		{
        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
			robotDrive.MecanumDrive_Cartesian(stick.GetX(), stick.GetY(), stick.GetZ());

			Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		}
	}

	void Robot::Autonomous() {

	}
START_ROBOT_CLASS(Robot)
