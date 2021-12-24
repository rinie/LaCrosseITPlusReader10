#ifndef _WH25_h
#define _WH25_h

#include "Arduino.h"
#include "WSBase.h"
#include "SensorBase.h"
#include "LaCrosse.h"
#define RESTORE_ANALYZE



class WH25 : public WSBase {
    float  Pressure;        // hPa
  public:
    static const byte FRAME_LENGTH = 12;
    static byte CalculateCRC(byte data[]);
    static void DecodeFrame(byte *bytes, struct WH25::Frame *frame);
    static String AnalyzeFrame(byte *data);
#ifndef RESTORE_ANALYZE
    static bool TryHandleData(byte *data);
    static String GetFhemDataString(byte *data);
    static bool IsValidDataRate(unsigned long dataRate);
#else
    static bool IsValidDataRate(unsigned long dataRate);
	static byte TryHandleData(byte *data, ulong dataRate, byte displayFormat = 0);
#endif

  protected:
    //static String BuildFhemDataString(struct WH25::Frame *frame, byte sensorType);
    //static String BuildKVDataString(struct WH25::Frame *frame, byte sensorType);


};


#endif

