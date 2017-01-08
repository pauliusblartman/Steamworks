#define PI 3.14159265
#include <math.h>
#include "WPILib.h"

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

class PIDLoop {

  AHRS gyro;
  Aimer aimer; //TJ's vision code

  PIDLoop() :
    gyro(I2C::Port::kMXP, 200) //update rate is 200Hz
    {}

  void pid() {
    PIDController pidAngle; //angle pid loop
    PIDController pidX; //xOffset pid loop
    PIDMoveSource angleSource;; //angle pid loop source variable
    PIDMoveSource xSource; //xOffset pid loop source variable
    PIDMoveOutput angleOutput; //angle pid loop output variable
    PIDMoveOutput xOutput; //angle pid loop source variable
    Timer timer;
    pidAngle(.1, .1, .1, angleSource, angleOutput, .006) //TODO: tune
    pidX(.1, .1, .1, xSource, xOutput, .006); //TODO: tune
    float angleOffset;
    float xOffset;
    float angleMaxError = 3; //TODO: arbitrary value, find actual - maybe make it a scalar based on the distance from the pin so that it accounts for actual difference in x over the y distance
    float xMaxError = 3; //TODO: arbitrary value, find actual
    float distance; //distance from the gear pin (hypotenuse)
    float offset; //-1 to 1 value of where the target is on the camera itself - needed for the math of calculating the angle offset (-1 is all the way left, 1 is all the way right)

    pidAngle.Enable();
    pidX.Enable();
    pidAngle.SetOutputRange(-.5, .5); //safety - mecanum drive turns really fast
    pidX.SetOutputRange(-1.0, 1.0); //safety

    distance = aimer.GetDistance();
    offset = aimer.GetOffset();

    angleOffset = gyro.GetYaw();
    xOffset = distance * (sin(gyro.GetYaw() * PI / 180) - (offset / 2)); //math is currently on my phone but will be put on google drive and in notebook

    angleSource.Set(angleOffset);
    xSource.Set(xOffset);
    timer.Reset();
    timer.Start();
    while ((fabs(pidAngle.Get())) > angleMaxError || (fabs(pidX.Get()) > xMaxError) && timer.Get() < 5) { //timer.Get() should be in seconds but will need to test to confirm
      angleOffset = gyro.GetYaw();
      xOffset = distance * (sin(gyro.GetYaw() * PI / 180) - (offset / 2));
      angleSource.Set(angleOffset);
      xSource.Set(xOffset);
      driveTrain.MecanumDrive_Cartesian(pidX.Get(), 0, pidAngle.Get(), gyro.GetYaw());
      Wait(.006); //max update rate of the gyro is .005 - TODO: will need to change based on how fast the camera updates
    }
  }
};
