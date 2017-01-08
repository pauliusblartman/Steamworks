#define PI 3.14159265
#include <math.h>
#include "WPILib.h"
#include "Constants.h"
#include "PIDMoveSource.h"
#include "PIDMoveOutput.h"

#ifndef SRC_PIDLOOP_H
#define SRC_PIDLOOP_H

class PIDLoop {

	AHRS gyro;
	Aimer aimer;
	Joystick joystick;
	RobotDrive driveTrain;
	PIDMoveSource angleSource;; //angle pid loop source variable
	PIDMoveSource xSource; //xOffset pid loop source variable
	PIDMoveOutput angleOutput; //angle pid loop output variable
	PIDMoveOutput xOutput; //angle pid loop source variable
	PIDController pidAngle; //angle pid loop
	PIDController pidX; //xOffset pid loop
	Timer timer;

public:
	PIDLoop(RobotDrive *driveTrain_, AHRS *gyro_, Joystick *joystick_);
	int runPID();
};

#endif
