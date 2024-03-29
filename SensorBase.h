#ifndef _SENSORBASE_h
#define _SENSORBASE_h

#include "Arduino.h"
#define RESTORE_ANALYZE
typedef unsigned long ulong;

class SensorBase {
public:
  static byte CalculateCRC(byte *data, byte len);
  static void SetDebugMode(boolean mode);

protected:
  static bool m_debug;

};

#endif

