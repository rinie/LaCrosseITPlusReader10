// Tested with JeeLink v3 (2012-02-11)
// polling RFM12B to decode FSK iT+ with a JeeNode/JeeLink from Jeelabs.
// Supported devices: see FHEM wiki
// info    : http://forum.jeelabs.net/node/110
//           http://fredboboss.free.fr/tx29/tx29_sw.php
//           http://www.f6fbb.org/domo/sensors/
//           http://www.mikrocontroller.net/topic/67273
//           benedikt.k org rinie,marf,joop 1 nov 2011, slightly modified by Rufik (r.markiewicz@gmail.com)
// Changelog: 2012-02-11: initial release 1.0
//            2014-03-14: I have this in SubVersion, so no need to do it here

#define PROGNAME         "LaCrosseITPlusReader"
#define PROGVERS         "10.1sJoRkr"

#include "SPI.h"
#include "RFM.h"
#include "SensorBase.h"
#include "LaCrosse.h"
#include "LevelSenderLib.h"
#include "EMT7110.h"
#include "WT440XH.h"
#include "TX38IT.h"
#include "JeeLink.h"
#include "Help.h"
#include "BMP180.h"
#include "WSBase.h"
#include "WS1080.h"
#include "TX22IT.h"
#include <Wire.h>
#include "InternalSensors.h"
#include "CustomSensor.h"
#include "WH24.h"
#include "WH25.h"
#include "W136.h"

// --- Configuration ---------------------------------------------------------------------------------------------------
#define RECEIVER_ENABLED       1                     // Set to 0 if you don't want to receive
#define USE_OLD_IDS            0                     // Set to 1 to use the old ID calcualtion

// The following settings can also be set from FHEM
#define ENABLE_ACTIVITY_LED    1         // <n>a     set to 0 if the blue LED bothers
ulong DATA_RATE_S1   = 17241ul;  // <n>c     use one of the possible data rates (for transmit on RFM #1)
bool DEBUG                   = 0;        // <n>d     set to 1 to see debug messages
#ifndef USE_SX127x
ulong INITIAL_FREQ   = 868350;   // <n>f     initial frequency in kHz (5 kHz steps, 860480 ... 879515)
#else
ulong INITIAL_FREQ   = 868300;   // <n>f     initial frequency in kHz (5 kHz steps, 860480 ... 879515)
#endif
int ALTITUDE_ABOVE_SEA_LEVEL = 0;        // <n>h     altituide above sea level
byte TOGGLE_MODE_R1          = 3;        // <n>m     bits 1: 17.241 kbps, 2 : 9.579 kbps, 4 : 8.842 kbps (for RFM #1)
byte TOGGLE_MODE_R2          = 3;        // <n>M     bits 1: 17.241 kbps, 2 : 9.579 kbps, 4 : 8.842 kbps (for RFM #2)
                                         // <n>o     set HF-parameter e.g. 50305o for RFM12 or 1,4o for RFM69
byte PASS_PAYLOAD            = 0;        // <n>p     transmitted the payload on the serial port 1: all, 2: only undecoded data
ulong DATA_RATE_R1   = 17241ul;  // <n>r     use one of the possible data rates (for RFM #1)
#ifdef USE_RFM2
ulong DATA_RATE_R2   = 9579ul;   // <n>R     use one of the possible data rates (for RFM #2)
#endif
                                         // <id,..>s send the bytes to the address id
uint16_t TOGGLE_INTERVAL_R1  = 0;        // <n>t     0=no toggle, else interval in seconds (for RFM #1)
#ifdef USE_RFM2
uint16_t TOGGLE_INTERVAL_R2  = 0;        // <n>T     0=no toggle, else interval in seconds (for RFM #2)
#endif
                                         // v        show version
                                         // x        test command
bool RELAY                   = 0;        // <n>y     if 1 all received packets will be retransmitted
byte displayFormat          = 0;        // <n>z     0 default, 1 AnalyzeFrame, 2 FhemDataString, 3 KVDataString


// --- Variables -------------------------------------------------------------------------------------------------------
ulong lastToggleR1 = 0;
#ifdef USE_RFM2
ulong lastToggleR2 = 0;
#endif
byte commandData[32];
byte commandDataPointer = 0;
#ifndef USE_SX127x
RFM rfm1(10, 2);
#ifdef USE_RFM2
RFM rfm2(11, 12, 13, 8, 2);
#endif
#else
// pins defined in pins_arduino.h
#if 0
// OLed
static const uint8_t OLED_SCL = 15;
static const uint8_t OLED_SDA = 4;
static const uint8_t OLED_RST = 16;

// LoRA
static const uint8_t LORA_SCK = 5;
static const uint8_t LORA_MOSI = 27;
static const uint8_t LORA_MISO = 19;
static const uint8_t LORA_CS = 18;
static const uint8_t LORA_RST = 14;
static const uint8_t LORA_IRQ = 26;
RFM rfm1(LORA_CS, LORA_IRQ, LORA_RST); // need RST?
RFM rfm2(LORA_CS, LORA_IRQ, LORA_RST); // need RST?
#endif
RFM rfm1(SS, DIO0, RST_LoRa); // need RST?
//RFM rfm1(18,26,14);
#ifdef USE_RFM2
RFM rfm2(SS, DIO0, RST_LoRa); // need RST?
#endif
#endif

JeeLink jeeLink;
InternalSensors internalSensors;

static ulong ConvertDataRate(ulong value) {
 ulong result = 0;
  switch (value) {
    case 0:
      result = 17241ul;
      break;
    case 1:
      result = 9579ul;
      break;
    case 2:
      result = 8842ul;
      break;
    default:
      result = value;
      break;
  }
  return result;
}

static void HandleSerialPort(char c) {
  static ulong value;
  ulong dataRate = 0;

  if (c == ',') {
    commandData[commandDataPointer++] = value;
    value = 0;
  }
  else if ('0' <= c && c <= '9') {
    value = 10 * value + c - '0';
  }
  else if (('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z')) {
    switch (c) {
    case 'd':
      // DEBUG
      SetDebugMode(value);
      break;
    case 'h':
      // height
      internalSensors.SetAltitudeAboveSeaLevel(value);
      break;
    case 'x':
      // Tests
      HandleCommandX(value);
      break;
    case 'a':
      // Activity LED
      jeeLink.EnableLED(value);
      break;
    case 'r':
    case 'R':
      // Data rate
      dataRate = ConvertDataRate(value);
      if (c == 'r') {
        DATA_RATE_R1 = dataRate;
        rfm1.SetDataRate(DATA_RATE_R1);
      }
#ifdef USE_RFM2
      else {
        if (rfm2.IsConnected()) {
          DATA_RATE_R2 = dataRate;
          rfm2.SetDataRate(DATA_RATE_R2);
        }
      }
#endif
      break;
    case 'c':
      // TX Data rate
      dataRate = ConvertDataRate(value);
      DATA_RATE_S1 = dataRate;
      break;
    case 'm':
      TOGGLE_MODE_R1 = value;
      break;
    case 'M':
      TOGGLE_MODE_R2 = value;
      break;
    case 'p':
      PASS_PAYLOAD = value;
      break;
    case 't':
      // Toggle data rate
      TOGGLE_INTERVAL_R1 = value;
      break;
#ifdef USE_RFM2
    case 'T':
      // Toggle data rate
      TOGGLE_INTERVAL_R2 = value;
      break;
#endif
    case 'v':
      // Version info
      HandleCommandV();
      break;

    case 's':
      // Send
      commandData[commandDataPointer] = value;
      HandleCommandS(commandData, ++commandDataPointer);
      commandDataPointer = 0;
      break;

    case 'o':
    case 'O':
      // Set HF parameter
      commandData[commandDataPointer] = value;
      HandleCommandO(c == 'O' ? 2 : 1, value, commandData, ++commandDataPointer);
      commandDataPointer = 0;
      break;

    case 'f':
      rfm1.SetFrequency(value);
      break;

#ifdef USE_RFM2
    case 'F':
      if (rfm2.IsConnected()) {
        rfm2.SetFrequency(value);
      }
      break;
#endif
    case 'y':
      RELAY = value;
      break;

    case 'z':
      displayFormat = value;
      break;

    default:
      HandleCommandV();
      #ifndef NOHELP
      Help::Show();
      #endif
      break;
    }
    value = 0;
  }
  else if (' ' < c && c < 'A') {
    HandleCommandV();
    #ifndef NOHELP
    Help::Show();
    #endif
  }
}

void SetDebugMode(boolean mode) {
  DEBUG = mode;
  LevelSenderLib::SetDebugMode(mode);
  WT440XH::SetDebugMode(mode);
  rfm1.SetDebugMode(mode);
#ifdef USE_RFM2
  rfm2.SetDebugMode(mode);
#endif
}

void HandleCommandO(byte rfmNbr, ulong value, byte *data, byte size) {
  // 50305o (is 0xC481) for RFM12 or 1,4o for RFM69
  if (size == 1 && rfm1.GetRadioType() == RFM::RFM12B) {
    if (rfmNbr == 1) {
      rfm1.SetHFParameter(value);
    }
#ifdef USE_RFM2
    else if (rfmNbr == 2) {
      rfm2.SetHFParameter(value);
    }
#endif
  }
  else if (size == 2 && rfm1.GetRadioType() == RFM::RFM69CW) {
    if (rfmNbr == 1) {
      rfm1.SetHFParameter(data[0], data[1]);
    }
#ifdef USE_RFM2
    else if (rfmNbr == 2) {
      rfm2.SetHFParameter(data[0], data[1]);
    }
#endif
  }


}

void HandleCommandS(byte *data, byte size) {
  rfm1.EnableReceiver(false);

  struct CustomSensor::Frame frame;
  frame.ID = data[0];
  frame.NbrOfDataBytes = size -1;

  for (int i = 0; i < frame.NbrOfDataBytes; i++) {
    frame.Data[i] = data[i+1];
  }

  CustomSensor::SendFrame(&frame, &rfm1, DATA_RATE_S1);


  rfm1.EnableReceiver(true);
}


// This function is for testing
void HandleCommandX(byte value) {
  //// A8 C0 58 5E 00 00 00 86 0A D8

  byte payload[10];
  //// A8 C0 58 63 01 03 00 88 08 C6
  //// ID : 8C, T = 8.8`C, relH = 99 % , Wvel = 0.3m / s, Wmax = 1.0m / s, Wdir = S, Rain = 40.8mm
  ////payload[0] = 0xA8;
  ////payload[1] = 0xC0;
  ////payload[2] = 0x58;
  ////payload[3] = 0x63;
  ////payload[4] = 0x01;
  ////payload[5] = 0x03;
  ////payload[6] = 0x00;
  ////payload[7] = 0x88;
  ////payload[8] = 0x08;
  ////payload[9] = 0xC6;

  ////WS1080::TryHandleData(payload);

}

void HandleCommandV() {
  Serial.print("\n[");
  Serial.print(PROGNAME);
  Serial.print('.');
  Serial.print(PROGVERS);

  Serial.print(" (");
  Serial.print(rfm1.GetRadioName());

  Serial.print(" f:");
  Serial.print(rfm1.GetFrequency());

  if (TOGGLE_INTERVAL_R1) {
    Serial.print(" t:");
    Serial.print(TOGGLE_INTERVAL_R1);
    Serial.print("~");
    Serial.print(TOGGLE_MODE_R1);
  }
  else {
    Serial.print(" r:");
    Serial.print(rfm1.GetDataRate());

  }
  Serial.print(")");

#ifdef USE_RFM2
  if(rfm2.IsConnected()) {
    Serial.print(" + (");
    Serial.print(rfm2.GetRadioName());
    Serial.print(" f:");
    Serial.print(rfm2.GetFrequency());
    if (TOGGLE_INTERVAL_R2) {
      Serial.print(" t:");
      Serial.print(TOGGLE_INTERVAL_R2);
      Serial.print("~");
      Serial.print(TOGGLE_MODE_R2);
    }
    else {
      Serial.print(" r:");
      Serial.print(rfm2.GetDataRate());

    }
    Serial.print(")");
  }
#endif
  if (internalSensors.HasBMP180()) {
    Serial.print(" + BMP180");
  }
  Serial.println(']');
}

void HandleReceivedData(RFM *rfm) {
  rfm->EnableReceiver(false);

  byte payload[PAYLOADSIZE];
  rfm->GetPayload(payload);
#if 0
  if (displayFormat) {
#ifdef ESP32
    WS1080::AnalyzeFrame(payload);
    TX22IT::AnalyzeFrame(payload);
    LaCrosse::AnalyzeFrame(payload);
    ////LevelSenderLib::AnalyzeFrame(payload);
    EMT7110::AnalyzeFrame(payload);
    TX38IT::AnalyzeFrame(payload);
    ////CustomSensor::AnalyzeFrame(payload);
    ////Serial.println();
#else
  Serial.println("Analyze Frames disabled");
#endif
  }
  else
#endif
  if (PASS_PAYLOAD == 1) {
    jeeLink.Blink(1);
    for (int i = 0; i < PAYLOADSIZE; i++) {
      Serial.print(payload[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  else {
    jeeLink.Blink(1);
    if (DEBUG) {
      Serial.print("\nEnd receiving, HEX raw data: ");
#ifdef CHECKHWCRC
      Serial.print("HW crc ok: ");
      Serial.print(rfm->IsCrcOk() ? "yes " : "no ");
#endif
#ifdef CHECKFIFOEMPTY
      Serial.print("HW payloadSize: ");
      Serial.print(rfm->GetPayloadSize());
      Serial.print(" ");
#endif
      for (int i = 0; i < 16; i++) {
        Serial.print(payload[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }

    byte frameLength = 0;
	ulong rfmDatarate = rfm->GetDataRate();
#ifndef RESTORE_ANALYZE
    // Try LaCrosse like TX29DTH
    if (LaCrosse::IsValidDataRate(rfm->GetDataRate()) && LaCrosse::TryHandleData(payload)) {
      frameLength = LaCrosse::FRAME_LENGTH;
    }
    // Try TX22IT (WS 1600)
    else if (TX22IT::IsValidDataRate(rfm->GetDataRate()) && TX22IT::TryHandleData(payload)) {
      frameLength = TX22IT::GetFrameLength(payload);
    }
    // Try WS 1080
    else if (WS1080::IsValidDataRate(rfm->GetDataRate()) && WS1080::TryHandleData(payload)) {
      frameLength = WS1080::FRAME_LENGTH;
    }

    // Try WH24
   else if (WH24::IsValidDataRate(rfm->GetDataRate()) && WH24::TryHandleData(payload)) {
      frameLength = WH24::FRAME_LENGTH;
    }
#else
    // Try LaCrosse like TX29DTH
    if (0 != (frameLength = LaCrosse::TryHandleData(payload, rfmDatarate, displayFormat))) {
      ;
    }
    // Try TX22IT (WS 1600)
    else if (0 != (frameLength = TX22IT::TryHandleData(payload, rfmDatarate, displayFormat))) {
      ;
    }
    // Try WS 1080
    else if (0 != (frameLength = WS1080::TryHandleData(payload, rfmDatarate, displayFormat))) {
      ;
    }
    // Try WH24
   else if (0 != (frameLength = WH24::TryHandleData(payload, rfmDatarate, displayFormat))) {
      //frameLength = WH24::FRAME_LENGTH;
      ;
    }
#endif
    // Try WH25
    else if (WH25::IsValidDataRate(rfm->GetDataRate()) && WH25::TryHandleData(payload)) {
      frameLength = WH25::FRAME_LENGTH;
    }

     // Try W136
    else if (W136::IsValidDataRate(rfm->GetDataRate()) && W136::TryHandleData(payload)) {
     frameLength = W136::FRAME_LENGTH;
    }

    // Try LevelSender
//    else if (LevelSenderLib::IsValidDataRate(rfm->GetDataRate()) && LevelSenderLib::TryHandleData(payload)) {
//      frameLength = LevelSenderLib::FRAME_LENGTH;
//    }

    // Try EMT7110
    else if (EMT7110::IsValidDataRate(rfm->GetDataRate()) && EMT7110::TryHandleData(payload)) {
      frameLength = EMT7110::FRAME_LENGTH;
    }

    // Try WT440XH
    else if (WT440XH::IsValidDataRate(rfm->GetDataRate()) && WT440XH::TryHandleData(payload)) {
      frameLength = WT440XH::FRAME_LENGTH;
    }
#ifndef RESTORE_ANALYZE
    // Try TX38IT
    else if (TX38IT::IsValidDataRate(rfm->GetDataRate()) && TX38IT::TryHandleData(payload)) {
      frameLength = TX38IT::FRAME_LENGTH;
    }
#else
    // Try TX22IT (WS 1600)
    else if (0 != (frameLength = TX38IT::TryHandleData(payload, rfmDatarate, displayFormat))) {
      ;
    }
#endif
    // Try CustomSensor
//    else if (CustomSensor::IsValidDataRate(rfm->GetDataRate()) && CustomSensor::TryHandleData(payload)) {
//      frameLength = CustomSensor::GetFrameLength(payload);
//    }
    else if (PASS_PAYLOAD == 2) {
      for (int i = 0; i < PAYLOADSIZE; i++) {
        Serial.print(payload[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }


    if (RELAY && frameLength > 0) {
      delay(64);
      rfm->SendArray(payload, frameLength);
      if (DEBUG) { Serial.println("Relayed"); }
    }

  }
  rfm->EnableReceiver(true);
}

void HandleDataRateToggle(RFM *rfm, ulong *lastToggle, ulong *dataRate, uint16_t interval, byte toggleMode) {
  if (interval > 0) {
    // After about 50 days millis() will overflow to zero
    if (millis() < *lastToggle) {
      *lastToggle = 0;
    }

    if (millis() > *lastToggle + interval * 1000) {
      // Bits 1: 17.241 kbps, 2 : 9.579 kbps, 4 : 8.842 kbps

      if (*dataRate == 8842ul) {
        if (toggleMode & 2) {
          *dataRate = 9579ul;
        }
        else if (toggleMode & 1) {
          *dataRate = 17241ul;
        }
      }
      else if (*dataRate == 9579ul) {
        if (toggleMode & 1) {
          *dataRate = 17241ul;
        }
        else if (toggleMode & 4) {
          *dataRate = 8842ul;
        }
      }
      else if (*dataRate == 17241ul) {
        if (toggleMode & 4) {
          *dataRate = 8842ul;
        }
        else if (toggleMode & 2) {
          *dataRate = 9579ul;
        }
      }

      rfm->SetDataRate(*dataRate);
      *lastToggle = millis();

    }
  }
}

// **********************************************************************
void loop(void) {
  // Handle the commands from the serial port
  // ----------------------------------------
  if (Serial.available()) {
    HandleSerialPort(Serial.read());
  }

  // Periodically send own sensor data
  // ---------------------------------
  internalSensors.TryHandleData();
  // Handle the data reception
  // -------------------------
  if (RECEIVER_ENABLED) {
    rfm1.Receive();
    if (rfm1.PayloadIsReady()) {
      HandleReceivedData(&rfm1);
    }

#ifdef USE_RFM2
    if(rfm2.IsConnected()) {
      rfm2.Receive();
      if (rfm2.PayloadIsReady()) {
        HandleReceivedData(&rfm2);
      }
    }
#endif
  }

  // Handle the data rate
  // --------------------
  HandleDataRateToggle(&rfm1, &lastToggleR1, &DATA_RATE_R1, TOGGLE_INTERVAL_R1, TOGGLE_MODE_R1);
#ifdef USE_RFM2
  HandleDataRateToggle(&rfm2, &lastToggleR2, &DATA_RATE_R2, TOGGLE_INTERVAL_R2, TOGGLE_MODE_R2);
#endif
}


void setup(void) {
  Serial.begin(57600);
  delay(200);

  if (DEBUG) {
    Serial.println("*** LaCrosse weather station wireless receiver for IT+ sensors ***");
  }
  SetDebugMode(DEBUG);
  LaCrosse::USE_OLD_ID_CALCULATION = USE_OLD_IDS;

  Wire.begin();
#ifdef USE_SPI_H
  SPI.begin();
#endif

  internalSensors.TryInitializeBMP180();
  internalSensors.SetAltitudeAboveSeaLevel(ALTITUDE_ABOVE_SEA_LEVEL);

  jeeLink.EnableLED(ENABLE_ACTIVITY_LED);
  lastToggleR1 = millis();

  rfm1.Begin(true);
  rfm1.InitializeLaCrosse();
  rfm1.SetFrequency(INITIAL_FREQ);
  rfm1.SetDataRate(DATA_RATE_R1);
  rfm1.EnableReceiver(true);

#ifdef USE_RFM2
  rfm2.Begin(false);
  if(rfm2.IsConnected()) {
    rfm2.InitializeLaCrosse();
    rfm2.SetFrequency(INITIAL_FREQ);
    rfm2.SetDataRate(DATA_RATE_R2);
    rfm2.EnableReceiver(true);
  }
#endif

  if (DEBUG) {
    Serial.println("Radio setup complete. Starting to receive messages");
  }
  // FHEM needs this information
  delay(1000);
  HandleCommandV();

}





