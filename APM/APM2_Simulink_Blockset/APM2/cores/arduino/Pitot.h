#ifndef __PITOT_H
#define __PITOT_H

#include "Arduino.h"
#include "FastSerial.h"

// Class Pitot
//
// Simple class to read dynamic pressure from pitot probe. APM library was not used because
// it would only calculate speed and not provide dynamic pressure. It also filtered the
// input which is not acceptable for our purpose. This has only been tested with the
// MPXV7002DP sensor and the APM2.
//
// R. Hartley, 2012

class Pitot
{
 public:
  Pitot(void);
  void calibrate(void);
  float get_dynamic_pressure();

 private:
  int   _offset;
  bool  _calibrated;
  float _scaling;
};

#endif /* __PITOT_H */
