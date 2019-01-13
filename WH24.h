#ifndef _WH24_h
#define _WH24_h

#include "Arduino.h"
#include "WSBase.h"
#include "SensorBase.h"
#include "LaCrosse.h"

#define RESTORE_ANALYZE

class WH24 : public WSBase {
    float  Pressure;        // hPa
  public:
    static const byte FRAME_LENGTH = 17;
    static byte CalculateCRC(byte data[]);
    static void DecodeFrame(byte *bytes, struct WH24::Frame *frame, bool fOnlyIfValid = true);
    static bool IsValidDataRate(unsigned long dataRate);
#ifndef RESTORE_ANALYZE
    static String AnalyzeFrame(byte *data);
    static bool TryHandleData(byte *data);
    static String GetFhemDataString(byte *data);
#else
	static byte TryHandleData(byte *data, ulong dataRate, byte displayFormat = 0);
#endif
  protected:
    //static String BuildFhemDataString(struct WH24::Frame *frame, byte sensorType);
    //static String BuildKVDataString(struct WH24::Frame *frame, byte sensorType);
 uint8_t crc8(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init);

};


#endif

