#ifndef _RFMXX_h
#define _RFMXX_h

#include "Arduino.h"

#ifdef ESP32
#define USE_SX127x
#endif
#define PAYLOADSIZE 64
#define IsRF69 (m_radioType == RFM69CW)

#define IsSX127x (m_radioType == SX127x)

class RFM {
public:
  enum RadioType {
    None = 0,
    RFM12B = 1,
    RFM69CW = 2,
    SX127x = 3
  };
#ifndef ESP32
  RFM(byte mosi, byte miso, byte sck, byte ss);
#else
  RFM(byte ss=SS, byte irqPin=2, byte reset=-1); // like USE_SPI_H
#endif
  void Begin(bool isPrimary);
  bool IsConnected();
  bool PayloadIsReady();
  void GetPayload(byte *data);
  void InitializeLaCrosse();
  void SendArray(byte *data, byte length);
  void SetDataRate(unsigned long dataRate);
  unsigned long GetDataRate();
  void SetFrequency(unsigned long kHz);
  unsigned long GetFrequency();
  void EnableReceiver(bool enable);
  void EnableTransmitter(bool enable);
  static byte CalculateCRC(byte data[], int len);
  void PowerDown();
  void SetDebugMode(boolean mode);
  RadioType GetRadioType();
  String GetRadioName();
  void Receive();
  void SetHFParameter(byte address, byte value);
  void SetHFParameter(unsigned short value);

private:
  RadioType m_radioType;
#ifndef ESP32
  byte m_mosi, m_miso, m_sck, m_ss, m_irq;
#else
  byte m_ss, m_irqPin, m_reset;
#endif
  bool m_debug;
  unsigned long m_dataRate;
  unsigned long m_frequency;
  byte m_payloadPointer;
  unsigned long m_lastReceiveTime;
  bool m_payloadReady;
  byte m_payload[PAYLOADSIZE];
  byte spi8(byte);
  unsigned short spi16(unsigned short value);
  byte ReadReg(byte addr);
  void WriteReg(byte addr, byte value);
  byte GetByteFromFifo();
  bool ClearFifo();
  void SendByte(byte data);

};

#ifdef USE_SX127x // use official semtech defines
#include "sx1276Regs-Fsk.h"
#else
#include "RFM69.h" // collides with sx1276 defines
#endif
#endif

