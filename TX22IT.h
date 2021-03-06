#ifndef _TX22IT_h
#define _TX22IT_h

#include "Arduino.h"
#include "WSBase.h"


class TX22IT : public WSBase {
public:
  static byte GetFrameLength(byte data[]);
  static byte CalculateCRC(byte data[]);
  static void EncodeFrame(struct WSBase::Frame *frame, byte bytes[4]);
  static void DecodeFrame(byte *bytes, struct WSBase::Frame *frame, bool fOnlyIfValid = true);
  static String AnalyzeFrame(byte *data);
  static bool IsValidDataRate(unsigned long dataRate);
#ifndef RESTORE_ANALYZE
  static bool TryHandleData(byte *data);
  static String GetFhemDataString(byte *data);
#else
	static byte TryHandleData(byte *data, ulong dataRate, byte displayFormat = 0);
#endif


protected:


};


#endif

