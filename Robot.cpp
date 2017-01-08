#include "Robot.h"

void moveToGearThreadFunction(bool *keepRunning, *PIDLoop pid) {
	while (*keepRunning == true) {
		if (pid->runPID() == 1) {
			*keepRunning = false;
		}
	}
}

Robot::Robot() :
		robotDrive(Constants::frontLeftDriveChannel, Constants::rearLeftDriveChannel, Constants::frontRightDriveChannel, Constants::rearRightDriveChannel),
		driveStick(Constants::driveStickChannel),
		operatorStick(Constants::operatorStickChannel),
    gyro(I2C::Port::kMXP, 200),
		pid(&robotDrive, &gyro, &operatorStick)
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
	bool gearMoveThreadRunBool = false;
	std::thread gearMoveThread(moveToGearThreadFunction, &gearMoveThreadRunBool, &pid);

	while (IsOperatorControl() && IsEnabled())
	{
		robotDrive.MecanumDrive_Cartesian(driveStick.GetX(), driveStick.GetY(), driveStick.GetZ(), gyro.GetYaw());

		if (operatorStick.GetRawButton(Constants::runGearMoveThreadButton)) {
			gearMoveThreadRunBool = true;
		} //should be able to reopen the thread after it's closed by the cancel button

		Wait(0.005); // wait 5ms to avoid hogging CPU cycles
	}

	gearMoveThreadRunBool = false;
	gearMoveThread.join();
	robotDrive.SetSafetyEnabled(true);
}

void Robot::Autonomous() {

}

START_ROBOT_CLASS(Robot)
