#include "PIDLoop.h"

PIDLoop::PIDLoop() {
  k_p_Angle = .013;
  k_i_Angle = .001;
  k_d_Angle = .001;
  p_Angle = 0;
  i_Angle = 0;
  d_Angle = 0;
  last_angle_error = 0;
  angleMaxError = 3;
  iteration_time = .005;
}

float PIDLoop::PIDAngle(float angleOffset, float desiredAngle) {
  //put in separate loop - not a while loop - keep checking and updating every runthrough of the normal loop - boolean for if this is running to stop you from manually moving the robot while the loop is running
  std::ofstream logger; logger.open("/var/loggerFile.txt", std::ofstream::out);
  logger << "Loop entered\n";

  angle_error = angleOffset - desiredAngle;
  angle_error = angle_error > 180 ? -(angle_error - 180) : angle_error;

  p_Angle = k_p_Angle * angle_error;
  //i_Angle += k_i_Angle * (angle_error * iteration_time);
  //d_Angle = k_d_Angle * ((angle_error - last_angle_error) / iteration_time);
  angleOutput = p_Angle + i_Angle + d_Angle;
  last_angle_error = angle_error;

  //angleOutput = (angle_error / 180) * .7 + .2;

  SmartDashboard::PutNumber("Desired Angle", desiredAngle);
  SmartDashboard::PutNumber("Angle Output", angleOutput);
  SmartDashboard::PutNumber("angleOffset", angleOffset);
  SmartDashboard::PutNumber("angle_error", angle_error);
  angleOutput = fabs(angleOutput) < .15 ? std::copysign(.15, angleOutput) : angleOutput;
  angleOutput = fabs(angleOutput) > .6 ? std::copysign(.6, angleOutput) : angleOutput;
  //angleOutput = angle_error < 0 ? angleOutput : -angleOutput;
  angleOutput = -angleOutput;
  logger << p_Angle << " " << angle_error << " " << angleOutput << "\n";
  frc::Wait(iteration_time);
  logger.close();

  return angleOutput;
}

float PIDLoop::PIDX() {
  float k_p_X = .05;
  float k_i_X = .05;
  float k_d_X = .05;
  float p_X;
  float i_X = 0;
  float d_X;
  float x_error;
  float last_x_error = 0;
  float xOffset;
  float xOutput;
  float xMaxError = 3;
  float distance;
  float cameraOffset;

  std::ofstream logger; logger.open("/var/loggerFile.txt", std::ofstream::out);
  logger << "Loop entered\n";
  distance = aimer.GetDistanceToGear();
  cameraOffset = aimer.GetOffset();
  xOffset = distance * (sin(angleOffset) - (cameraOffset / 2)); //math is currently on my phone but will be put on google drive and in notebook


  x_error = xOffset;

  p_X = k_p_X * x_error;
  i_X += k_i_X * (x_error * iteration_time);
  d_X = k_d_X * ((x_error - last_x_error) / iteration_time);
  xOutput = p_X + i_X + d_X;
  last_x_error = x_error;

  distance = aimer.GetDistanceToGear();
  cameraOffset = aimer.GetOffset();
  xOffset = distance * (sin(angleOffset) - (cameraOffset / 2)); //math is currently on my phone but will be put on google drive and in notebook

  x_error = xOffset;

  xOutput = 0;

  frc::Wait(iteration_time);

  logger.close();

  return 0;
}
