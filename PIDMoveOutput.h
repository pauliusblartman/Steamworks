#ifndef SRC_PIDMOVEOUTPUT_H
#define SRC_PIDMOVEOUTPUT_H

class PIDMoveSource : public PIDSource {

  double output;

public:
  void PIDWrite(double value);
};

#endif
