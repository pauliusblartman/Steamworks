#define PI 3.14159265
#include <math.h>
#include "WPILib.h"
#include "Constants.h"

class PIDMoveSource : PIDSource { //extends PIDSource - for the PIDController
  double moveValue; //gyro value
  public: double PIDGet() { //created by WPILib - to give the value to the PIDController
    return moveValue;
  }
  public: void Set(double value) { //allows us to set the value for the PIDController
    moveValue = value;
  }
};

class PIDMoveOutput : PIDOutput {
  double output;
  public: void PIDWrite(double value) { //created by WPILib - allows the controller to store an output value to output
    output = value;
  }
};

class PIDLoop { //TODO: make a .h file

  AHRS gyro;
  Aimer aimer; //TJ's vision code
  RobotDrive driveTrain;

  PIDLoop(RobotDrive *driveTrain_, AHRS *gyro_) :
    gyro(gyro_), //update rate is 200Hz
    aimer(),
    driveTrain(driveTrain_)
    {}

  void pid() {
    PIDController pidAngle; //angle pid loop
    PIDController pidX; //xOffset pid loop
    PIDMoveSource angleSource;; //angle pid loop source variable
    PIDMoveSource xSource; //xOffset pid loop source variable
    PIDMoveOutput angleOutput; //angle pid loop output variable
    PIDMoveOutput xOutput; //angle pid loop source variable
    Timer timer;
    pidAngle(Constants::angleP, Constants::angleI, Constants::angleD, angleSource, angleOutput, Constants::gearPIDIterationTime) //TODO: tune | also may need to flip the negative on the p value depending on which direction positive mecanum rotates
    pidX(Constants::xOffsetP, Constants::xOffsetI, Constants::xOffsetD, xSource, xOutput, Constants::gearPIDIterationTime); //TODO: tune | also may need to add a negative to the p value depending on which direction +x is in mecanum
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
      angleOffset = gyro.GetYaw();
      xOffset = distance * (sin(gyro.GetYaw() * PI / 180) - (offset / 2));
      angleSource.Set(angleOffset);
      xSource.Set(xOffset);
      driveTrain->MecanumDrive_Cartesian(pidX.Get(), 0, pidAngle.Get(), gyro.GetYaw());
      Wait(Constants::gearPIDIterationTime); //TODO: change based on camera update rate
      //TODO: this code should also be adjusted (eventually) to move the robot in the y direction if it doesn't
    }
  }
};
