#include "WPILib.h"

#include "Constants.h"

#include <math.h>
#include <thread>
#include <fstream>

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

class Robot : public SampleRobot {

	RobotDrive robotDrive;	// robot drive system
	Joystick stick;			// only joystick

public:
	Robot();
	void RobotInit();
	void OperatorControl();
	void Autonomous();
};

#endif /* SRC_ROBOT_H_ */
