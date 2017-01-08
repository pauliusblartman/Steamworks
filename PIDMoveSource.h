#ifndef SRC_PIDMOVESOURCE_H
#define SRC_PIDMOVESOURCE_H

class PIDMoveSource : public PIDSource {

  double moveValue;

public:
  double PIDGet();
  void Set(double value);
};

#endif
