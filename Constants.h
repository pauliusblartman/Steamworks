#ifndef SRC_CONSTANTS_H
#define SRC_CONSTANTS_H

namespace Constants {

  //Gear PID loop
  static constexpr float angleP = -.01;
  static constexpr float angleI = .01;
  static constexpr float angleD = .01;
  static constexpr float xOffsetP = .01;
  static constexpr float xOffsetI = .01;
  static constexpr float xOffsetD = .01;
  static constexpr float angleMaxError = 3;
  static constexpr float xMaxError = 3;
  static constexpr float gearPIDIterationTime = .006; //max update rate of the gyro is .005 - TODO: will need to change based on how fast the camera updates
  static constexpr float angleOutputMin = -.5;
  static constexpr float angleOutputMax = .5;
  static constexpr float xOutputMin = -1.0;
  static constexpr float xOutputMax = 1.0;

};
#endif
