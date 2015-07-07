#include "TimeStats.h"

// Class TimeStats
//
// R. Hartley, 2012

TimeStats::TimeStats(void)
{
  _time_at_mark_ms = 0;
  _time_at_mark_us = 0;
  _specified_frame_rate_ms = 0;
}

void TimeStats::mark(void)
{
  _time_at_mark_us = micros();
  _time_at_mark_ms = millis();
}

void TimeStats::set_specified_frame_rate_ms(unsigned long ts)
{
  _specified_frame_rate_ms = ts;
}

unsigned long TimeStats::get_specified_frame_rate_ms(void)
{
  return _specified_frame_rate_ms;
}

unsigned long TimeStats::get_delta_ms(void)
{
  return (millis() - _time_at_mark_ms);
}


unsigned long TimeStats::get_delta_us(void)
{
  return (micros() - _time_at_mark_us);
}

unsigned long TimeStats::get_millis(void)
{
  return millis();
}

unsigned long TimeStats::get_micros(void)
{
  return micros();
}



