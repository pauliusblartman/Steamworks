#include "WPILib.h"
#include "Constants.h"
#include "PIDLoop.h"
#include <math.h>
#include <fstream>

#ifndef SRC_PIDLOOP_H
#define SRC_PIDLOOP_H

class PIDLoop {

  float k_p_Angle;
  float k_i_Angle;
  float k_d_Angle;
  float p_Angle;
  float i_Angle;
  float d_Angle;
  float angle_error;
  float last_angle_error;
  float angleOffset;
  float angleOutput;
  float angleMaxError;
  float iteration_time;

public:
	PIDLoop();
	float PIDAngle(float yaw, float desiredAngle);
	float PIDX();
};

#endif /* SRC_ROBOT_H_ */
