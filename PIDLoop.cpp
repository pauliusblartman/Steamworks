#include "PIDLoop.h"

PIDLoop::PIDLoop(RobotDrive *driveTrain_, AHRS *gyro_, Joystick *joystick) :
  gyro(gyro_), //update rate is 200Hz
  aimer(),
  joystick(joystick_),
  driveTrain(driveTrain_),
  pidAngle(Constants::angleP, Constants::angleI, Constants::angleD, angleSource, angleOutput, Constants::gearPIDIterationTime), //TODO: tune | also may need to flip the negative on the p value depending on which direction positive mecanum rotates
  pidX(Constants::xOffsetP, Constants::xOffsetI, Constants::xOffsetD, xSource, xOutput, Constants::gearPIDIterationTime), //TODO: tune | also may need to add a negative to the p value depending on which direction +x is in mecanum
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

  distance = aimer.GetDistance();
  offset = aimer.GetOffset();

  angleOffset = gyro.GetYaw();
  xOffset = distance * (sin(gyro.GetYaw() * PI / 180) - (offset / 2)); //math is currently on my phone but will be put on google drive and in notebook

  angleSource.Set(angleOffset);
  xSource.Set(xOffset);

  timer.Reset();
  timer.Start();

  while (((fabs(pidAngle.Get()) > Constants::angleMaxError) || (fabs(pidX.Get()) > Constants::xMaxError)) && timer.Get() < 5) { //timer.Get() should be in seconds but will need to test to confirm
    if (joystick.GetRawButton(Constants::cancelGearMoveThreadButton)) {
      pidAngle.Disable();
      pidX.Disable();
      return 1;
    }
    angleOffset = gyro.GetYaw();
    xOffset = distance * (sin(gyro.GetYaw() * PI / 180) - (offset / 2));
    angleSource.Set(angleOffset);
    xSource.Set(xOffset);
    driveTrain->MecanumDrive_Cartesian(pidX.Get(), 0, pidAngle.Get(), gyro.GetYaw());
    Wait(Constants::gearPIDIterationTime); //TODO: change based on camera update rate
    //TODO: this code should also be adjusted (eventually) to move the robot in the y direction if it doesn't
  }

  pidAngle.Disable();
  pidX.Disable();

  return 1;
}
