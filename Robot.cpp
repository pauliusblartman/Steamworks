#include "Robot.h"
#include "WPILib.h"

/*void moveToGearThreadFunction(bool *keepRunning, PIDLoop *pid) {
	while (*keepRunning == true) {
		if (pid->runPID() == 1) {
			*keepRunning = false;
		}
	}
}*/

Robot::Robot() :
		robotDrive(Constants::frontLeftDriveChannel, Constants::rearLeftDriveChannel, Constants::frontRightDriveChannel, Constants::rearRightDriveChannel),
		driveStick(Constants::driveStickChannel),
		operatorStick(Constants::operatorStickChannel),
		gyro(I2C::Port::kMXP, 200),
		pid(),
		aimer()
{
	robotDrive.SetExpiration(0.1);
	robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, false);
	robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
	robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
	robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
}

void Robot::RobotInit() {}

/**
 * Runs the motors with Mecanum drive.
 */
void Robot::OperatorControl()
{
	robotDrive.SetSafetyEnabled(false);
	//bool gearMoveThreadRunBool = false;
	//std::thread gearMoveThread(moveToGearThreadFunction, &gearMoveThreadRunBool, &pid); //thread not needed yet - might need to be implemented later
	gyro.ZeroYaw();
	float driveX;
	float driveY;
	float driveZ;
	float angle;
	float angleOutput = 0; //pid loop output

	while (IsOperatorControl() && IsEnabled())
	{
		angleOutput = 0; //reset output so that if the pid loop isn't being called it's not reserved from the last time it's called
		angle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
		if(driveStick.GetPOV() != -1) { //turn to angle 0, 90, 180, 270
			angleOutput = pid.PIDAngle(angle, driveStick.GetPOV()); //call pid loop
		}

		driveX = fabs(driveStick.GetX()) < .05 ? 0.0 : driveStick.GetX(); //deadzones - need to figure out deadzones better (RecycleRush code might have them)
		driveY = fabs(driveStick.GetY()) < .05 ? 0.0 : driveStick.GetY(); //TODO: make getrawaxis w/ numbers instead of getx, gety, getz
		driveZ = fabs(driveStick.GetZ()) < .5 ? 0.0 : driveStick.GetZ() * .33; //Z axis deadzone is huge
		robotDrive.MecanumDrive_Cartesian(driveX, driveY, driveZ + angleOutput);
/*		if (operatorStick.GetRawButton(Constants::runGearMoveThreadButton)) {
			gearMoveThreadRunBool = true;
		} //should be able to reopen the thread after it's closed by the cancel button*/ //thread code - not needed right now
		SmartDashboard::PutNumber("Angle to gear", aimer.GetAngleToGear());
		frc::Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		SmartDashboard::PutNumber("Angle", angle);
		SmartDashboard::PutBoolean("Is Rotating", gyro.IsRotating());
		SmartDashboard::PutNumber("Requested Update rate", gyro.GetRequestedUpdateRate());
		SmartDashboard::PutNumber("Actual Update rate", gyro.GetActualUpdateRate());
		SmartDashboard::PutNumber("getPOV", driveStick.GetPOV());
	}
	//gearMoveThreadRunBool = false;
	//gearMoveThread.join();
	robotDrive.SetSafetyEnabled(true);
}

void Robot::Autonomous() {

}

START_ROBOT_CLASS(Robot)
