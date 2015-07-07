#ifndef __TIMESTATS_H
#define __TIMESTATS_H

#include "Arduino.h"


// Class TimeStats

//
// R. Hartley, 2012

class TimeStats
{
 public:
  TimeStats(void);
  void mark(void);
  unsigned long get_delta_ms();
  unsigned long get_delta_us();
  unsigned long get_millis();
  unsigned long get_micros();
  void set_specified_frame_rate_ms(unsigned long ts);
  unsigned long get_specified_frame_rate_ms();
  
 private:
  unsigned long _time_at_mark_ms;
  unsigned long _time_at_mark_us;
  unsigned long _specified_frame_rate_ms;
  
};

#endif /* __TIMESTATS_H */
