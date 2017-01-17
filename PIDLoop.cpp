#include "PIDLoop.h"


PIDLoop::PIDLoop(RobotDrive *driveTrain_, Joystick *joystick_, AHRS *gyro_) :
	gyro(I2C::Port::kMXP),
	aimer(),
	joystick(Constants::operatorStickChannel),
	driveTrain(Constants::frontLeftDriveChannel, Constants::rearLeftDriveChannel, Constants::frontRightDriveChannel, Constants::rearRightDriveChannel),
	timer()
{}


int PIDLoop::runPID() {
	float k_p_Angle = .001;
	float k_i_Angle = .001;
	float k_d_Angle = .001;
	float k_p_X = .05;
	float k_i_X = .05;
	float k_d_X = .05;
	float p_Angle;
	float i_Angle = 0;
	float d_Angle;
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

	distance = aimer.GetDistanceToGear();
	cameraOffset = aimer.GetOffset();
	angleOffset = gyro.GetYaw() + 180;
	xOffset = distance * (sin(angleOffset) - (cameraOffset / 2)); //math is currently on my phone but will be put on google drive and in notebook


	float desiredAngle = 90;
	angle_error = angleOffset - desiredAngle;

	//angle_error = angleOffset - aimer.GetAngleToGear();
	x_error = xOffset;
	while (((fabs(angle_error) > angleMaxError) || (fabs(x_error) > xMaxError)) && failsafe < 500) {

		p_Angle = k_p_Angle * angle_error;
		i_Angle += k_i_Angle * (angle_error * iteration_time);
		d_Angle = k_d_Angle * ((angle_error - last_angle_error) / iteration_time);
		angleOutput = p_Angle + i_Angle + d_Angle;
		last_angle_error = angle_error;

		p_X = k_p_X * x_error;
		i_X += k_i_X * (x_error * iteration_time);
		d_X = k_d_X * ((x_error - last_x_error) / iteration_time);
		xOutput = p_X + i_X + d_X;
		last_x_error = x_error;

		distance = aimer.GetDistanceToGear();
		cameraOffset = aimer.GetOffset();
		angleOffset = gyro.GetYaw() + 180;
		xOffset = distance * (sin(angleOffset) - (cameraOffset / 2)); //math is currently on my phone but will be put on google drive and in notebook


		angle_error = angleOffset - desiredAngle;

		//angle_error = angleOffset - aimer.GetAngleToGear();
		x_error = xOffset;
		SmartDashboard::PutNumber("Angle Output", angleOutput);
		xOutput = 0;
		driveTrain.MecanumDrive_Cartesian(xOutput, 0, angleOutput, gyro.GetYaw() + 180);

		frc::Wait(iteration_time);
		failsafe++;
	}
	return 1;
}















/*PIDLoop::PIDLoop(RobotDrive *driveTrain_, Joystick *joystick_, AHRS *gyro_) :
  gyro(I2C::Port::kMXP),
  //gyro(gyro_), //update rate is 200Hz
  aimer(),
  //joystick(joystick_),
  joystick(Constants::operatorStickChannel),
  driveTrain(Constants::frontLeftDriveChannel, Constants::rearLeftDriveChannel, Constants::frontRightDriveChannel, Constants::rearRightDriveChannel),
  //driveTrain(driveTrain_),
  pidAngle(.1, .1, .1, gyro, angleOutput, .05),
  pidX(.1, .1, .1, xSource, xOutput, .05),
  //pidAngle(Constants::angleP, Constants::angleI, Constants::angleD, gyro, angleOutput, Constants::gearPIDIterationTime), //TODO: tune | also may need to flip the negative on the p value depending on which direction positive mecanum rotates
  //pidX(Constants::xOffsetP, Constants::xOffsetI, Constants::xOffsetD, xSource, xOutput, Constants::gearPIDIterationTime), //TODO: tune | also may need to add a negative to the p value depending on which direction +x is in mecanum
  timer()
  {}

int PIDLoop::runPID() {
  float angleOffset;
  float xOffset;
  float distance; //distance from the gear pin (hypotenuse)
  float offset; //-1 to 1 value of where the target is on the camera itself - needed for the math of calculating the angle offset (-1 is all the way left of the camera view, 1 is all the way right)

  pidAngle.Enable();
  pidX.Enable();
  pidAngle.SetOutputRange(Constants::angleOutputMin, Constants::angleOutputMax); //safety - mecanum drive turns really fast
  pidX.SetOutputRange(Constants::xOutputMin, Constants::xOutputMax); //safety

  distance = aimer.GetDistanceToGear();
  offset = aimer.GetOffset();

  angleOffset = gyro.GetYaw();
  xOffset = distance * (sin(gyro.GetYaw() * PI / 180) - (offset / 2)); //math is currently on my phone but will be put on google drive and in notebook

//  angleSource.Set(angleOffset);
// xSource.Set(xOffset);

  timer.Reset();
  timer.Start();

  while ((((fabs(pidAngle.Get()) > Constants::angleMaxError) || (fabs(pidX.Get()) > Constants::xMaxError)) && timer.Get() < 5) && !joystick.GetRawButton(Constants::cancelGearMoveThreadButton)) { //timer.Get() should be in seconds but will need to test to confirm
    angleOffset = gyro.GetYaw();
    xOffset = distance * (sin(gyro.GetYaw() * PI / 180) - (offset / 2));
    //angleSource.Set(angleOffset);
    //xSource.Set(xOffset);
    driveTrain.MecanumDrive_Cartesian(pidX.Get(), 0, pidAngle.Get(), gyro.GetYaw());
    frc::Wait(Constants::gearPIDIterationTime); //TODO: change based on camera update rate
    //TODO: this code should also be adjusted (eventually) to move the robot in the y direction if it doesn't
  }
  
  pidAngle.Disable();
  pidX.Disable();

  return 1;
}*/
