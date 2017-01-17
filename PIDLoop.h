#define PI 3.14159265
#include <math.h>
#include "Constants.h"
//#include "PIDMoveSource.h"
//#include "PIDMoveOutput.h"
#include "Aimer.h"
#include "AHRS.h"
#include "WPILib.h"

#ifndef SRC_PIDLOOP_H
#define SRC_PIDLOOP_H

class PIDLoop {

	AHRS gyro;
	Aimer aimer;
	frc::Joystick joystick;
	frc::RobotDrive driveTrain;
	/*PIDMoveSource angleSource;
	PIDMoveSource xSource; //xOffset pid loop source variable
	PIDMoveOutput angleOutput; //angle pid loop output variable
	PIDMoveOutput xOutput; //angle pid loop source variable
	PIDController pidAngle; //angle pid loop
	PIDController pidX; //xOffset pid loop*/
	frc::Timer timer;



public:
	PIDLoop(RobotDrive *driveTrain_, Joystick *joystick_, AHRS *gyro_);
	int runPID();
};

#endif
