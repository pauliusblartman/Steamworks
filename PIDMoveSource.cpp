#include "PIDMoveSource.h"

double PIDMoveSource::PIDGet() { //created by WPILib - to give the value to the PIDController
  return moveValue;
}
void PIDMoveSource::Set(double value) { //allows us to set the value for the PIDController
  moveValue = value;
}
