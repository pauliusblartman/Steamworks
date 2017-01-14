#include "Robot.h"
#include "WPILib.h"

void moveToGearThreadFunction(bool *keepRunning, PIDLoop *pid) {
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
		gyro(I2C::Port::kMXP),
		pid(&robotDrive, &operatorStick, &gyro),
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
	bool gearMoveThreadRunBool = false;
	std::thread gearMoveThread(moveToGearThreadFunction, &gearMoveThreadRunBool, &pid);
	gyro.ZeroYaw();
	float driveX;
	float driveY;
	float driveZ;
	float angle;


	float k_p_Angle = .013;
	float k_i_Angle = .001;
	float k_d_Angle = .001;
	float k_p_X = .05;
	float k_i_X = .05;
	float k_d_X = .05;
	float p_Angle;
	float i_Angle = 0;
	float d_Angle = 0;
	float p_X;
	float i_X = 0;
	float d_X;
	float angle_error;
	float x_error;
	float last_angle_error = 0;
	float last_x_error = 0;
	float angleOffset;
	float xOffset;
	float angleOutput;
	float xOutput;
	float angleMaxError = 3;
	float xMaxError = 3;
	float iteration_time = .005;
	float distance;
	float cameraOffset;
	int failsafe = 0;


	while (IsOperatorControl() && IsEnabled())
	{


		if(driveStick.GetPOV() != -1) {
			std::ofstream logger; logger.open("/var/loggerFile.txt", std::ofstream::out);
			logger << "Loop entered\n";
			float desiredAngle = driveStick.GetPOV();
			distance = aimer.GetDistanceToGear();
			cameraOffset = aimer.GetOffset();
			angleOffset = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
			xOffset = distance * (sin(angleOffset) - (cameraOffset / 2)); //math is currently on my phone but will be put on google drive and in notebook


			angle_error = angleOffset - desiredAngle;
			//angle_error = angleOffset - aimer.GetAngleToGear();
			x_error = xOffset;
			SmartDashboard::PutBoolean("loop", true);
			while (((fabs(angle_error) > angleMaxError) || (fabs(x_error) > xMaxError)) && failsafe < 1000) {

				p_Angle = k_p_Angle * angle_error;
				//i_Angle += k_i_Angle * (angle_error * iteration_time);
				//d_Angle = k_d_Angle * ((angle_error - last_angle_error) / iteration_time);
				angleOutput = p_Angle + i_Angle + d_Angle;
				last_angle_error = angle_error;

				p_X = k_p_X * x_error;
				i_X += k_i_X * (x_error * iteration_time);
				d_X = k_d_X * ((x_error - last_x_error) / iteration_time);
				xOutput = p_X + i_X + d_X;
				last_x_error = x_error;

				distance = aimer.GetDistanceToGear();
				cameraOffset = aimer.GetOffset();
				angleOffset = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
				xOffset = distance * (sin(angleOffset) - (cameraOffset / 2)); //math is currently on my phone but will be put on google drive and in notebook

				angle_error = angleOffset - desiredAngle;

				//angleOutput = (angle_error / 180) * .7 + .2;

				//angle_error = angleOffset - aimer.GetAngleToGear();
				x_error = xOffset;
				SmartDashboard::PutNumber("Angle Output", angleOutput);
				SmartDashboard::PutNumber("angleOffset", angleOffset);
				SmartDashboard::PutNumber("angle_error", angle_error);
				xOutput = 0;
				//angleOutput = angleOutput < .15 ? std::copysign(.15, angleOutput) : angleOutput;
				robotDrive.MecanumDrive_Cartesian(xOutput, 0, -angleOutput, gyro.GetYaw() + 180);
				logger << p_Angle << " " << angle_error << " " << angleOutput << "\n";
				frc::Wait(iteration_time);
				failsafe++;
				if (driveStick.GetRawButton(2)) {
					break;
				}
			}
			angle_error = 20;
			SmartDashboard::PutBoolean("loop", false);
			logger.close();
		}

		driveX = fabs(driveStick.GetX()) < .05 ? 0.0 : driveStick.GetX();
		driveY = fabs(driveStick.GetY()) < .05 ? 0.0 : driveStick.GetY();
		driveZ = fabs(driveStick.GetZ()) < .5 ? 0.0 : driveStick.GetZ() * .33;
		robotDrive.MecanumDrive_Cartesian(driveX, driveY, driveZ);

		if (operatorStick.GetRawButton(Constants::runGearMoveThreadButton)) {
			gearMoveThreadRunBool = true;
		} //should be able to reopen the thread after it's closed by the cancel button
		SmartDashboard::PutNumber("Angle to gear", aimer.GetAngleToGear());
		frc::Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		angle = gyro.GetYaw() < 0 ? 360 + gyro.GetYaw() : gyro.GetYaw();
		SmartDashboard::PutNumber("Angle", angle);
		SmartDashboard::PutBoolean("Is Rotating", gyro.IsRotating());
		SmartDashboard::PutNumber("Update rate", gyro.GetActualUpdateRate());
		SmartDashboard::PutNumber("getPOV", driveStick.GetPOV());
	}
	gearMoveThreadRunBool = false;
	gearMoveThread.join();
	robotDrive.SetSafetyEnabled(true);
}

void Robot::Autonomous() {

}

START_ROBOT_CLASS(Robot)
