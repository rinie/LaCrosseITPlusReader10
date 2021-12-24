#ifndef _WSBASE_h
#define _WSBASE_h

#include "Arduino.h"
#include "SensorBase.h"

class WSBase : public SensorBase {
public:
  struct Frame {
    byte  Header;
    byte  ID;
    byte  CRC;
    bool  NewBatteryFlag;
    bool  LowBatteryFlag;
    bool  ErrorFlag;
    bool  IsValid;

    bool  HasTemperature;
    bool  HasHumidity;
    bool  HasRain;
    bool  HasWindSpeed;
    bool  HasWindDirection;
    bool  HasWindGust;
    bool  HasPressure;
    bool  HasUV;
    bool  HasstrikesTotal;
    bool  HasstrikesDistance;

    double Temperature;    // �C
    double Rain;           // mm
    double WindDirection;  // Degree
    double  WindSpeed;     // m/s
    double  WindGust;      // m/s
    double  Pressure;        // hPa
    byte  Humidity;       // %rH
    double UV;
    int16_t strikesDistance ;
    uint16_t strikesTotal ;
  };
  enum SensorType {
	TX22IT = 1,
	NodeSensor = 2,
    WH1080 = 3,
    LG = 4,
    WH25 = 5,
    W136 = 6,
    WH24 = 7
  };

protected:
  static String BuildKVDataString(struct Frame *frame, SensorType sensorType);
  static String BuildFhemDataString(struct Frame *frame, byte sensorType);
  static String AddWord(word value, bool hasValue);
  static String AddByte(byte value, bool hasValue);
  static float DecodeValue(byte q1, byte q2, byte q3);
  static String AnalyzeFrame(byte *data, Frame *frame, byte frameLength, String prefix);
};

#endif

