#include "Pitot.h"

// Class Pitot
//
// Simple class to read dynamic pressure from pitot probe. APM library was not used because
// it would only calculate speed and not provide dynamic pressure. It also filtered the
// input which is not acceptable for our purpose. This has only been tested with the
// MPXV7002DP sensor and the APM2.
//
// R. Hartley, 2012

Pitot::Pitot(void)
{
  _offset = 0;
  _calibrated = false;
  _scaling = 0.1019797; // (5/1024)*1*20.8854342;

    /*
    (1 V/kPa) from sensor
    (5 V / 1024) from adc
    20.8854342 psf per kpa
  */
}

void Pitot::calibrate(void)
{
  int sum = 0;
  int c;

  for (c = 0; c < 50; c++) {
    sum += analogRead(0);
  }

  _offset = sum/c;
  _calibrated = true;
}

float Pitot::get_dynamic_pressure()
{
  int raw;

  if (_calibrated)
    {
      raw = analogRead(0) - _offset;
      return (((float)raw) * _scaling);
    }
  else
    {
      return 0;
    }
}

