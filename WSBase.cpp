#include "WSBase.h"

String WSBase::BuildFhemDataString(struct Frame *frame, byte sensorType) {
  /* Format
  OK WS 60  1   4   193 52    2 88  4   101 15  20   ID=60  21.7?C  52%rH  600mm  Dir.: 112.5?  Wind:15m/s  Gust:20m/s
  OK WS ID  XXX TTT TTT HHH RRR RRR DDD DDD SSS SSS GGG GGG FFF PPP PPP
  |  |  |   |   |   |   |   |   |   |   |   |   |   |   |   |-- Flags *
  |  |  |   |   |   |   |   |   |   |   |   |   |   |   |------ WindGust * 10 LSB (0.0 ... 50.0 m/s)           FF/FF = none
  |  |  |   |   |   |   |   |   |   |   |   |   |   |---------- WindGust * 10 MSB
  |  |  |   |   |   |   |   |   |   |   |   |   |-------------- WindSpeed  * 10 LSB(0.0 ... 50.0 m/s)          FF/FF = none
  |  |  |   |   |   |   |   |   |   |   |   |------------------ WindSpeed  * 10 MSB
  |  |  |   |   |   |   |   |   |   |   |---------------------- WindDirection * 10 LSB (0.0 ... 365.0 Degrees) FF/FF = none
  |  |  |   |   |   |   |   |   |   |-------------------------- WindDirection * 10 MSB
  |  |  |   |   |   |   |   |   |------------------------------ Rain LSB (0 ... 9999 mm)                       FF/FF = none
  |  |  |   |   |   |   |   |---------------------------------- Rain MSB
  |  |  |   |   |   |   |-------------------------------------- Humidity (1 ... 99 %rH)                        FF = none
  |  |  |   |   |   |------------------------------------------ Temp * 10 + 1000 LSB (-40 ... +60 ?C)          FF/FF = none
  |  |  |   |   |---------------------------------------------- Temp * 10 + 1000 MSB
  |  |  |   |-------------------------------------------------- Sensor type (1=TX22IT, 2=NodeSensor, 3=WS1080)
  |  |  |------------------------------------------------------ Sensor ID (1 ... 63)
  |  |--------------------------------------------------------- fix "WS"
  |------------------------------------------------------------ fix "OK"

  * Flags: 128  64  32  16  8   4   2   1
  |   |   |
  |   |   |-- New battery
  |   |------ ERROR
  |---------- Low battery
  */

  String pBuf = "";

  // Check if data is in the valid range
  bool isValid = true;
  if (frame->ErrorFlag) {
    isValid = false;
  }
  if (frame->HasTemperature && (frame->Temperature < -40.0 || frame->Temperature > 59.9)) {
    isValid = false;
  }
  if (frame->HasHumidity && (frame->Humidity < 1 || frame->Humidity > 100)) {
    isValid = false;
  }

  if (isValid) {
    pBuf += "OK WS ";
    pBuf += frame->ID;
    pBuf += " ";
    pBuf += sensorType;

    // add temperature
    pBuf += AddWord(frame->Temperature * 10 + 1000, frame->HasTemperature);

    // add humidity
    pBuf += AddByte(frame->Humidity, frame->HasHumidity);

    // add rain
    pBuf += AddWord(frame->Rain, frame->HasRain);

    // add wind direction
    pBuf += AddWord(frame->WindDirection * 10, frame->HasWindDirection);

    // add wind speed
    pBuf += AddWord(frame->WindSpeed * 10, frame->HasWindSpeed);

    // add gust
    pBuf += AddWord(frame->WindGust * 10, frame->HasWindGust);


    // add Flags
    byte flags = 0;
    if (frame->NewBatteryFlag) {
      flags += 1;
    }
    if (frame->ErrorFlag) {
      flags += 2;
    }
    if (frame->LowBatteryFlag) {
      flags += 4;
    }
    pBuf += AddByte(flags, true);

    // add pressure
    if (frame->HasPressure) {
    pBuf += AddWord(frame->Pressure * 10, frame->HasPressure);
    }
  }

  return pBuf;
}


String WSBase::AddWord(word value, bool hasValue) {
  String result;

  if (!hasValue) {
    value = 0xFFFF;
  }

  result += ' ';
  result += (byte)(value >> 8);
  result += ' ';
  result += (byte)(value);

  return result;
}

String WSBase::AddByte(byte value, bool hasValue) {
  String result;
  result += ' ';
  result += hasValue ? value : 0xFF;

  return result;
}

float WSBase::DecodeValue(byte q1, byte q2, byte q3) {
  float result = 0;

  result += q1 * 100;
  result += q2 * 10;
  result += q3;

  return result;
}

String WSBase::AnalyzeFrame(byte *data, Frame *frame, byte frameLength, String prefix) {
  String result;

  // Show the raw data bytes
  result += prefix;
  result += " [";
  for (int i = 0; i < frameLength; i++) {
	if (data[i] < 16) {
	    result += String(0, HEX);
	}
    result += String(data[i], HEX);
    if (i < frameLength) {
      result += " ";
    }
  }
  result += "]";

  // CRC
  if (!frame->IsValid) {
    result += " CRC:WRONG";
  }
  else {
    result += " CRC:OK";

    // Start
    result += " S:";
    result += String(frame->Header, HEX);

    // Sensor ID
    result += " ID:";
    result += String(frame->ID, HEX);

    if (frame->HasTemperature) {
		// Temperature
		result += " Temp:";
		if (frame->HasTemperature) {
		  result += frame->Temperature;
		}
		else {
		  result += "---";
		}
	}
    if (frame->HasHumidity) {
		// Humidity
		result += " Hum:";
		if (frame->HasHumidity) {
		  result += frame->Humidity;
		}
		else {
		  result += "---";
		}
	}
    if (frame->HasPressure) {
		result += " Press:";
		if (frame->Pressure) {
		  result += frame->Pressure;
		}
		else {
		  result += "---";
		}
    }

    if (frame->HasRain) {
		// Rain
		result += " Rain:";
		if (frame->HasRain) {
		  result += frame->Rain;
		}
		else {
		  result += "---";
		}
	}

    if (frame->HasWindSpeed) {
		// Wind speed
		result += " Wind:";
	    if (frame->HasWindSpeed) {
		  result += frame->WindSpeed;
		  result += "m/s";
		}
		else {
		  result += "---";
		}
    }
    if (frame->HasWindDirection) {
		// Wind direction
		result += " from:";
		if (frame->HasWindDirection) {
		  result += frame->WindDirection;
		}
		else {
		  result += "---";
		}
	}
    if (frame->HasWindGust) {
		// Wind gust
		result += " Gust:";
		if (frame->HasWindGust) {
		  result += frame->WindGust;
		  result += " m/s";
		}
		else {
		  result += "---";
		}
	}
    // New battery flag
    result += " NewBatt:";
    result += frame->NewBatteryFlag ? 1 : 0;

    // Low battery flag
    result += " LowBatt:";
    result += frame->LowBatteryFlag ? 1 : 0;

    // Error flag
    result += " Error:";
    result += frame->ErrorFlag ? 1 : 0;

    // CRC
    result += " CRC:";
    result += String(frame->CRC, HEX);
  }

  return result;
}

String WSBase::BuildKVDataString(struct Frame *frame, SensorType sensorType) {
  // KeyValue example
  // Format:  KV <Type> <Address> <Key>=<Value>,<Key>=<Value>,<Key>=<Value>, ...
  // Example: KV ADDON 01 Voltage=3.3,UpTime=100
  // -> LGW will send it as KeyValueProtocol to FHEM

  String pBuf = "";
  String pBuf2 = "";
   String pBuf3;
  String sensorTypeName = "";

  // Check if data is in the valid range
  bool isValid = true;
  if (frame->ErrorFlag) {
    isValid = false;
  }

  switch (sensorType) {
    case 1: sensorTypeName = "TX22IT"; break;
    case 2: sensorTypeName = "NodeSensor"; break;
    case 3: sensorTypeName = "WH1080"; break;
    case 4: sensorTypeName = "LG"; break;
    case 5: sensorTypeName = "WH25"; break;
    case W136: sensorTypeName = "W136"; break;
    case 7: sensorTypeName = "WH24"; break;
    default: sensorTypeName = "XX"; break;
  }


  if (isValid) {
    pBuf += "OK VALUES ";
    pBuf += sensorTypeName;
    pBuf += " ";
    pBuf += frame->ID;
    pBuf += " ";

  //  pBuf += "Header=";
  //  pBuf += frame->Header;
  //  pBuf += ",";

    // add temperature
    if (frame->HasTemperature && (frame->Temperature < -40.0 || frame->Temperature > 59.9)) {
      isValid = false;
    } else {
      pBuf += "Temperature=";
      pBuf += frame->Temperature;
      pBuf += ",";
    }
    // add humidity
    if (frame->HasHumidity && (frame->Humidity < 1 || frame->Humidity > 100)) {
      isValid = false;
    } else {
      pBuf += "Humidity=";
      pBuf += frame->Humidity;
      pBuf += ",";
    }
    // add pressure
    if (frame->HasPressure) {
      if ( (frame->Pressure > 500 || frame->Pressure < 2000)) {
        pBuf += "Pressure=";
        pBuf += frame->Pressure;
        pBuf += ",";
      }
    }
    // add rain
    if (frame->HasRain) {
      pBuf += "Rain=";
      pBuf += frame->Rain;
      pBuf += ",";
    }
    // add wind speed
    if (frame->HasWindSpeed) {
      pBuf += "WindSpeed=";
      pBuf += frame->WindSpeed;
      pBuf += ",";
    }
    // add wind direction
    if (frame->HasWindDirection) {
      pBuf += "WindDirection=";
      pBuf += frame->WindDirection;
      pBuf += ",";
    }

    // add gust
    if (frame->HasWindGust) {
      pBuf += "WindGust=";
      pBuf += frame->WindGust;
      pBuf += ",";
    }
    // add UV
    if (frame->HasUV) {
      pBuf += "UV=";
      pBuf += frame->UV;
      pBuf += ",";
    }
    // add strikesDistance
    if (frame->HasstrikesDistance) {
      pBuf += "strikesDistance=";
      pBuf += frame->strikesDistance;
      pBuf += ",";
    }
    // add strikesTotal
    if (frame->HasstrikesTotal) {
      pBuf += "strikesTotal=";
      pBuf += frame->strikesTotal;
      pBuf += ",";
    }
    // add Flags
    if (frame->NewBatteryFlag) {
      pBuf += "NewBatteryFlag=";
      pBuf += frame->NewBatteryFlag;
      pBuf += ",";
    }

    if (frame->ErrorFlag) {
      pBuf += "ErrorFlag=";
      pBuf += frame->ErrorFlag;
      pBuf += ",";
    }
    if (frame->LowBatteryFlag) {
      pBuf += "LowBatteryFlag=";
      pBuf += frame->LowBatteryFlag;
      pBuf += ",";
    }

  }
   return pBuf;
}

