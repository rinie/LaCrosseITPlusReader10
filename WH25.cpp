#include "WH25.h"

/*
  WH25  17.241 kbps  868.3 MHz
  -------------------------------
  Extracted data:
            ?I IT TT HH PP PP CC BB
   aa 2d d4 e5 02 72 28 27 21 c9 bb aa
    0  1  2  3  4  5  6  7  8  9 10
             0  1  2  3  4  5  6  7
   II = Sensor ID (based on 2 different sensors). Does not change at battery change.
   T TT = Temperature (+40*10)
   HH = Humidity
   PP PP = Pressure (*10)
   CC = Checksum of previous 6 bytes (binary sum truncated to 8 bit)
   BB = Bitsum (XOR) of the 6 data bytes (high and low nibble exchanged)

   OK VALUES WH25 50 Header=14,Temperature=xx.xx,Humidity=69,Pressure=1013.80, [E3 2A 8D 45 27 9A A0 AA AA AA 00 7D]
   OK VALUES WH25 43 Header=14,Temperature=23.80,Humidity=56,Pressure=1012.10, [E2 B2 7E 38 27 89 FA AA AA AA 00 5F]

rlt_433: WH32B is the same as WH25 but two packets in one transmission of {971} and XOR sum missing.

FRAME_LENGTH = 12;
   CC = Checksum of previous 6 bytes (binary sum truncated to 8 bit)
   BB = Bitsum (XOR) of the 6 data bytes (high and low nibble exchanged)
Data layout:
    aa 2d d4 e5 02 72 28 27 21 c9 bb aa
             ?I IT TT HH PP PP CC BB

- I: 8 bit Sensor ID (based on 2 different sensors). Does not change at battery change.
- B: 1 bit low battery indicator
- F: 1 bit invalid reading indicator
- T: 10 bit Temperature (+40*10), top two bits are flags
- H: 8 bit Humidity
- P: 16 bit Pressure (*10)
- C: 8 bit Checksum of previous 6 bytes (binary sum truncated to 8 bit)
- B: 8 bit Bitsum (XOR) of the 6 data bytes (high and low nibble exchanged)
WH32B is the same as WH25 but two packets in one transmission of {971} and XOR sum missing.
                                              0  1  2  3  4  5  6  7  8  B
wh25                                        [E3 2A 8D 45 27 9A A0 AA AA AA 00 7D]
09:39:49.825 -> End receiving, HEX raw data: ED 92 53 2E 27 11 38 00 00 00 00 00 0 0 0 0

09:40:49.796 -> End receiving, HEX raw data: ED 92 53 2E 27 10 37 0 0 0 0 0 0 0 0 0

09:41:49.790 -> End receiving, HEX raw data: ED 92 53 2E 27 10 37 0 0 0 0 0 0 0 0 0

09:42:49.803 -> End receiving, HEX raw data: ED 92 53 2E 27 12 39 0 0 0 0 0 0 0 0 0

09:43:49.820 -> End receiving, HEX raw data: ED 92 53 2E 27 10 37 0 0 0 0 0 0 0 0 0

*/

void WH25::DecodeFrame(byte *bytes, struct Frame *frame) {

  uint8_t tempkorr = 0;
  frame->IsValid = true;
  frame->ID = 0;
  frame->NewBatteryFlag = false;
  frame->LowBatteryFlag = false;
  frame->ErrorFlag = false;

  frame->HasTemperature = true;
  frame->HasHumidity = true;
  frame->HasRain = false;
  frame->HasWindSpeed = false;
  frame->HasWindDirection = false;
  frame->HasWindGust = false;
  frame->HasPressure = true;
  frame->HasUV = false;
  frame->HasstrikesTotal = false;
  frame->HasstrikesDistance = false;


  frame->Header = bytes[0] >> 4;
  if (frame->Header == 0xE) {
    frame->IsValid = true;
  }
  else {
    frame->IsValid = false;
  }

  if (frame->IsValid) {
    // Verify checksum
    uint8_t checksum = 0, bitsum = 0;

    for (size_t n = 0; n <= 5; ++n) {
      checksum += bytes[n];
      bitsum ^= bytes[n];
    }

    bitsum = (bitsum << 4) | (bitsum >> 4);     // Swap nibbles

	if (checksum == bytes[6] &&  bytes[9] == 0x00) { //WH32b. no bitsum
/*
      Serial.print("WH32: ");
      Serial.print(bytes[6], HEX);
      Serial.print(' ');
      Serial.print(checksum, HEX);
      Serial.print(' ');
      Serial.println(bitsum, HEX);
*/
		  frame->IsValid = true;
	      tempkorr = 0;
	}
	else {
		if (checksum == bytes[6] && bytes[9] == 0xAA ) {
		  tempkorr = 0;
		  frame->IsValid = true;
		} else {
		  frame->IsValid = false;
		}
		if (frame->IsValid && bitsum == bytes[7]) { // WH25A ab Release 20/14 andere Temp-Darstellung
		  //if (checksum != bytes[6] || bitsum != bytes[7]) & ( bytes[9] != 0xAA) {
		  tempkorr = 1;
		} else {
		  tempkorr = 0;
		}
	}
  }

  if (frame->IsValid) {
	  /*
	     // Decode data
	      uint8_t id        = ((b[0]&0x0f) << 4) | (b[1] >> 4);
	      int low_battery   = (b[1] & 0x08) >> 3;
	      //int invalid_flag  = (b[1] & 0x04) >> 2;
	      int temp_raw      = (b[1] & 0x03) << 8 | b[2]; // 0x7ff if invalid
	      double temperature = (temp_raw - 400) * 0.1f;    // range -40.0-60.0 C
	      uint8_t humidity  = b[3];
	      int pressure_raw  = (b[4] << 8 | b[5]);
    	double pressure    = pressure_raw * 0.1f;
    */
    frame->ID = ((bytes[0] << 4) | (bytes[1] >> 4)) & 0xFF;

    frame->NewBatteryFlag = false;
    frame->ErrorFlag = false;
    frame->LowBatteryFlag = true;
    frame->LowBatteryFlag = ((bytes[1] & 0x08) >> 3) != 0;


    // Temperature (ï¿½C)
    int temp = (bytes[1] & 0x07) << 8 | bytes[2]; // 0x7ff if invalid

   /* int temp = ((bytes[1] & 0xF) << 8) | bytes[2];
    if (tempkorr == 1) {
      temp = ((bytes[1] & 0x2) << 8) | bytes[2]; // ab WH25A 20/14 ist das Protokoll anders !?
    }
    */
    frame->Temperature = (temp - 400) * 0.1f;  // range -40.0-60.0 C

    // Humidity (%rH)
    frame->Humidity = bytes[3];

    frame->Pressure = ((bytes[4] << 8) | bytes[5]) * 0.1;
    frame->CRC=bytes[6];

  }
}


#ifndef RESTORE_ANALYZE
String WH25::AnalyzeFrame(byte *data) {
  struct Frame frame;
  DecodeFrame(data, &frame);

  byte frameLength = WH25::FRAME_LENGTH;

  return WSBase::AnalyzeFrame(data, &frame, frameLength, "WH25");
}

String WH25::GetFhemDataString(byte *data) {
  String fhemString2 = "";
  // if ((data[0] >> 4) == 0x0E) {
  struct Frame frame2;
  DecodeFrame(data, &frame2);
  if (frame2.IsValid) {
    //fhemString2 = BuildFhemDataString(&frame2, 5);
    fhemString2 = BuildKVDataString(&frame2, WSBase::WH25);
  }
  //}

  return fhemString2;
}

bool WH25::TryHandleData(byte *data) {
  String fhemString = GetFhemDataString(data);

  if (fhemString.length() > 0) {
    Serial.println(fhemString);
  }

  return fhemString.length() > 0;
}



bool WH25::IsValidDataRate(unsigned long dataRate) {
  return dataRate == 17241ul;
}
#else
bool WH25::IsValidDataRate(unsigned long dataRate) {
  return dataRate == 17241ul;
}

byte WH25::TryHandleData(byte *data, ulong dataRate, byte displayFormat)
{
  struct Frame frame;
  bool fOnlyIfValid = true;
  String frameDataString;

  if (!IsValidDataRate(dataRate)) {
	  // frameLength = 0;
	  return 0;
  }

  DecodeFrame(data, &frame);
  if (frame.IsValid || !fOnlyIfValid) {
	  if (displayFormat == 1) {
		  frameDataString = WSBase::AnalyzeFrame(data, &frame, WH25::FRAME_LENGTH, "WH25");
	  }
	  else if (frame.IsValid) {
		  if (displayFormat == 2) {
		    frameDataString = BuildFhemDataString(&frame, WSBase::WH25);
		  }
		  else { // default
		  	frameDataString = BuildKVDataString(&frame, WSBase::WH25);
	     }
	  }
      Serial.println(frameDataString);
  }
  return frame.IsValid ? WH25::FRAME_LENGTH : 0;
}

#endif