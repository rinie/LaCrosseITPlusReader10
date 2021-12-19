#include "RFM.h"

void RFM::Receive() {
  if (IsRF69) {
    if (ReadReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) {
      for (int i = 0; i < PAYLOADSIZE; i++) {
        byte bt = GetByteFromFifo();
        m_payload[i] = bt;
      }
      m_payloadReady = true;
    }
  }
  else {
    bool hasData = false;
    digitalWrite(m_ss, LOW);
    asm("nop");
    asm("nop");
    if (digitalRead(m_miso)) {
      hasData = true;
    }
    digitalWrite(m_ss, HIGH);

    if (hasData) {
      m_payload[m_payloadPointer++] = GetByteFromFifo();
      m_lastReceiveTime = millis();
    }

    if ((m_payloadPointer > 0 && millis() > m_lastReceiveTime + 50) || m_payloadPointer >= 32) {
      m_payloadReady = true;
    }
  }
}

void RFM::GetPayload(byte *data) {
  m_payloadReady = false;
  m_payloadPointer = 0;
  for (int i = 0; i < PAYLOADSIZE; i++) {
    data[i] = m_payload[i];
  }
}


void RFM::SetDataRate(unsigned long dataRate) {
  m_dataRate = dataRate;
  if (IsRF69) {
    word r = ((32000000UL + (m_dataRate / 2)) / m_dataRate);
    WriteReg(0x03, r >> 8);
    WriteReg(0x04, r & 0xFF);
  }
  else {
    byte bt = (byte)(round(344828.0 / m_dataRate)) - 1;
    RFM::spi16(0xC600 | bt);
  }
}

void RFM::SetFrequency(unsigned long kHz) {
  m_frequency = kHz;

  if (IsRF69) {
    unsigned long f = (((kHz * 1000) << 2) / (32000000L >> 11)) << 6;
    WriteReg(0x07, f >> 16);
    WriteReg(0x08, f >> 8);
    WriteReg(0x09, f);
  }
  else {
    RFM::spi16(40960 + (m_frequency - 860000) / 5);
  }

}

void RFM::EnableReceiver(bool enable) {
  if (enable) {
    if (IsRF69) {
      WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
    }
    else {
      spi16(0x82C8);
      spi16(0xCA81);
      spi16(0xCA83);
    }
  }
  else {
    if (IsRF69) {
      WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
    }
    else {
      spi16(0x8208);
    }
  }
  ClearFifo();
}

void RFM::EnableTransmitter(bool enable) {
  if (enable) {
    if (IsRF69) {
      WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
    }
    else {
      spi16(0x8238);
    }
  }
  else {
    if (IsRF69) {
      WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
    }
    else {
      spi16(0x8208);
    }
  }
}

byte RFM::GetByteFromFifo() {
  return IsRF69 ? ReadReg(0x00) : (byte)spi16(0xB000);
}

bool RFM::PayloadIsReady() {
  return m_payloadReady;
}


bool RFM::ClearFifo() {
  if (IsRF69) {
    WriteReg(REG_IRQFLAGS2, 16);
  }
  else {
    for (byte i = 0; i < PAYLOADSIZE; i++) {
      spi16(0xB000);
    }
  }

}

void RFM::PowerDown() {
  if (IsRF69) {
    WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
  }
  else {
    spi16(0x8201);
  }
}

void RFM::InitializeLaCrosse() {
  if (m_debug) {
    Serial.print("Radio is: ");
    Serial.println(GetRadioName());
  }

  digitalWrite(m_ss, HIGH);
  EnableReceiver(false);

  if (IsRF69) {
#if 1
    /* 0x01 */ WriteReg(REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY);
    /* 0x02 */ WriteReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00);
    /* 0x05 */ WriteReg(REG_FDEVMSB, RF_FDEVMSB_60000);
    /* 0x06 */ WriteReg(REG_FDEVLSB, RF_FDEVLSB_60000);
    /* 0x11 */ WriteReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111);
    /* 0x13 */ WriteReg(REG_OCP, RF_OCP_OFF);
    /* 0x19 */ WriteReg(REG_RXBW, /* RF_RXBW_DCCFREQ_010 | */ RF_RXBW_MANT_16 | RF_RXBW_EXP_2);
    /* 0x28 */ WriteReg(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
    /* 0x29 */ WriteReg(REG_RSSITHRESH, 220);
#if 1
    /* 0x2D */ WriteReg(REG_PREAMBLELSB, 2);
#endif
    /* 0x2E */ WriteReg(REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0);
    /* 0x2F */ WriteReg(REG_SYNCVALUE1, 0x2D);
    /* 0x30 */ WriteReg(REG_SYNCVALUE2, 0xD4);
#if 0
    /* 0x37 */ WriteReg(REG_PACKETCONFIG1, RF_PACKET1_CRCAUTOCLEAR_OFF);
#else
			#define RF_PACKET1_FORMAT_FIXED                       0x00  // Default
			#define RF_PACKET1_FORMAT_VARIABLE                    0x80

			#define RF_PACKET1_DCFREE_OFF                         0x00  // Default
			#define RF_PACKET1_DCFREE_MANCHESTER                  0x20
			#define RF_PACKET1_DCFREE_WHITENING                   0x40

			#define RF_PACKET1_CRC_ON                             0x10  // Default
			#define RF_PACKET1_CRC_OFF                            0x00

			#define RF_PACKET1_CRCAUTOCLEAR_ON                    0x00  // Default
			#define RF_PACKET1_CRCAUTOCLEAR_OFF                   0x08

			#define RF_PACKET1_ADRSFILTERING_OFF                  0x00  // Default
			#define RF_PACKET1_ADRSFILTERING_NODE                 0x02
			#define RF_PACKET1_ADRSFILTERING_NODEBROADCAST        0x04

 			WriteReg(REG_PACKETCONFIG1,
                 RF_PACKET1_CRC_OFF |
                 RF_PACKET1_CRCAUTOCLEAR_OFF);
#endif
#if 0
    /* 0x38 */ WriteReg(REG_PAYLOADLENGTH, PAYLOADSIZE);
#else
    /* 0x38 */ WriteReg(REG_PAYLOADLENGTH, 17);
#endif
    /* 0x3C */ WriteReg(REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE);
#if 0
    /* 0x3D */ WriteReg(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);
#else
    /* 0x3D */ WriteReg(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_1BIT | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);
#endif
    /* 0x6F */ WriteReg(REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0);
#else
// RFM69 registers

#define RFM69_REG_00_FIFO                             0x00
#define RFM69_REG_01_OPMODE                           0x01
#define RFM69_REG_02_MODUL                            0x02
#define RFM69_REG_03_BITRATE_MSB                      0x03
#define RFM69_REG_04_BITRATE_LSB                      0x04
#define RFM69_REG_05_FDEV_MSB                         0x05
#define RFM69_REG_06_FDEV_LSB                         0x06
#define RFM69_REG_07_FRF_MSB                          0x07
#define RFM69_REG_08_FRF_MID                          0x08
#define RFM69_REG_09_FRF_LSB                          0x09
#define RFM69_REG_0A_OSC1                             0x0A
#define RFM69_REG_0B_AFC_CTRL                         0x0B
#define RFM69_REG_0D_LISTEN1                          0x0D
#define RFM69_REG_0E_LISTEN2                          0x0E
#define RFM69_REG_0F_LISTEN3                          0x0F
#define RFM69_REG_10_VERSION                          0x10
#define RFM69_REG_11_PA_LEVEL                         0x11
#define RFM69_REG_12_PA_RAMP                          0x12
#define RFM69_REG_13_OCP                              0x13
#define RFM69_REG_18_LNA                              0x18
#define RFM69_REG_19_RX_BW                            0x19
#define RFM69_REG_1A_AFC_BW                           0x1A
#define RFM69_REG_1B_OOK_PEAK                         0x1B
#define RFM69_REG_1C_OOK_AVG                          0x1C
#define RFM69_REG_1D_OOF_FIX                          0x1D
#define RFM69_REG_1E_AFC_FEI                          0x1E
#define RFM69_REG_1F_AFC_MSB                          0x1F
#define RFM69_REG_20_AFC_LSB                          0x20
#define RFM69_REG_21_FEI_MSB                          0x21
#define RFM69_REG_22_FEI_LSB                          0x22
#define RFM69_REG_23_RSSI_CONFIG                      0x23
#define RFM69_REG_24_RSSI_VALUE                       0x24
#define RFM69_REG_25_DIO_MAPPING1                     0x25
#define RFM69_REG_26_DIO_MAPPING2                     0x26
#define RFM69_REG_27_IRQ_FLAGS1                       0x27
#define RFM69_REG_28_IRQ_FLAGS2                       0x28
#define RFM69_REG_29_RSSI_THRESHOLD                   0x29
#define RFM69_REG_2A_RX_TIMEOUT1                      0x2A
#define RFM69_REG_2B_RX_TIMEOUT2                      0x2B
#define RFM69_REG_2C_PREAMBLE_MSB                     0x2C
#define RFM69_REG_2D_PREAMBLE_LSB                     0x2D
#define RFM69_REG_2E_SYNC_CONFIG                      0x2E
#define RFM69_REG_2F_SYNCVALUE1                       0x2F
#define RFM69_REG_30_SYNCVALUE2                       0x30
#define RFM69_REG_31_SYNCVALUE3                       0x31
#define RFM69_REG_32_SYNCVALUE4                       0x32
#define RFM69_REG_33_SYNCVALUE5                       0x33
#define RFM69_REG_34_SYNCVALUE6                       0x34
#define RFM69_REG_35_SYNCVALUE7                       0x35
#define RFM69_REG_36_SYNCVALUE8                       0x36
#define RFM69_REG_37_PACKET_CONFIG1                   0x37
#define RFM69_REG_38_PAYLOAD_LENGTH                   0x38
#define RFM69_REG_39_NODE_ADDRESS                     0x39
#define RFM69_REG_3A_BROADCAST_ADDRESS                0x3A
#define RFM69_REG_3B_AUTOMODES                        0x3B
#define RFM69_REG_3C_FIFO_THRESHOLD                   0x3C
#define RFM69_REG_3D_PACKET_CONFIG2                   0x3D
#define RFM69_REG_3E_AES_KEY_BYTE1                    0x3E
#define RFM69_REG_3F_AES_KEY_BYTE2                    0x3F
#define RFM69_REG_40_AES_KEY_BYTE3                    0x40
#define RFM69_REG_41_AES_KEY_BYTE4                    0x41
#define RFM69_REG_42_AES_KEY_BYTE5                    0x42
#define RFM69_REG_43_AES_KEY_BYTE6                    0x43
#define RFM69_REG_44_AES_KEY_BYTE7                    0x44
#define RFM69_REG_45_AES_KEY_BYTE8                    0x45
#define RFM69_REG_46_AES_KEY_BYTE9                    0x46
#define RFM69_REG_47_AES_KEY_BYTE10                   0x47
#define RFM69_REG_48_AES_KEY_BYTE11                   0x48
#define RFM69_REG_49_AES_KEY_BYTE12                   0x49
#define RFM69_REG_4A_AES_KEY_BYTE13                   0x4A
#define RFM69_REG_4B_AES_KEY_BYTE14                   0x4B
#define RFM69_REG_4C_AES_KEY_BYTE15                   0x4C
#define RFM69_REG_4D_AES_KEY_BYTE16                   0x4D
#define RFM69_REG_4E_TEMP1                            0x4E
#define RFM69_REG_4F_TEMP2                            0x4F
#define RFM69_REG_58_TEST_LNA                         0x58
#define RFM69_REG_5A_TEST_PA1                         0x5A
#define RFM69_REG_5C_TEST_PA2                         0x5C
#define RFM69_REG_6F_TEST_DAGC                        0x6F
#define RFM69_REG_71_TEST_AFC                         0x71

// RegOpMode
#define RF_OPMODE_SEQUENCER_OFF                       0x80
#define RF_OPMODE_SEQUENCER_ON                        0x00  // Default

#define RF_OPMODE_LISTEN_ON                           0x40
#define RF_OPMODE_LISTEN_OFF                          0x00  // Default

#define RF_OPMODE_LISTENABORT                         0x20

#define RF_OPMODE_SLEEP                               0x00
#define RF_OPMODE_STANDBY                             0x04  // Default
#define RF_OPMODE_SYNTHESIZER                         0x08
#define RF_OPMODE_TRANSMITTER                         0x0C
#define RF_OPMODE_RECEIVER                            0x10

// RegDataModul
#define RFMODULMODE_PACKET                            0x00  // Default
#define RFMODULMODE_CONTINUOUS                        0x40
#define RFMODULMODE_CONTINUOUSNOBSYNC                 0x60

#define RFMODUL_MODULATIONTYPE_FSK                    0x00  // Default
#define RFMODUL_MODULATIONTYPE_OOK                    0x08

#define RFMODUL_MODULATIONSHAPING_00                  0x00  // Default
#define RFMODUL_MODULATIONSHAPING_01                  0x01
#define RFMODUL_MODULATIONSHAPING_10                  0x02
#define RFMODUL_MODULATIONSHAPING_11                  0x03

// RegOsc1
#define RF_OSC1_RCCAL_START                           0x80
#define RF_OSC1_RCCAL_DONE                            0x40

// RegAfcCtrl
#define RF_AFCLOWBETA_ON                              0x20
#define RF_AFCLOWBETA_OFF                             0x00    // Default

// RegLowBat
#define RF_LOWBAT_MONITOR                             0x10
#define RF_LOWBAT_ON                                  0x08
#define RF_LOWBAT_OFF                                 0x00  // Default

#define RF_LOWBAT_TRIM_1695                           0x00
#define RF_LOWBAT_TRIM_1764                           0x01
#define RF_LOWBAT_TRIM_1835                           0x02  // Default
#define RF_LOWBAT_TRIM_1905                           0x03
#define RF_LOWBAT_TRIM_1976                           0x04
#define RF_LOWBAT_TRIM_2045                           0x05
#define RF_LOWBAT_TRIM_2116                           0x06
#define RF_LOWBAT_TRIM_2185                           0x07

// RegListen1
#define RF_LISTEN1_RESOL_64                           0x50
#define RF_LISTEN1_RESOL_4100                         0xA0  // Default
#define RF_LISTEN1_RESOL_262000                       0xF0

#define RF_LISTEN1_CRITERIA_RSSI                      0x00  // Default
#define RF_LISTEN1_CRITERIA_RSSIANDSYNC               0x08

#define RF_LISTEN1_END_00                             0x00
#define RF_LISTEN1_END_01                             0x02  // Default
#define RF_LISTEN1_END_10                             0x04

// RegListen2
#define RF_LISTEN2_COEFIDLE_VALUE                     0xF5 // Default

// RegListen3
#define RF_LISTEN3_COEFRX_VALUE                       0x20 // Default

// RegPaLevel
#define RF_PALEVEL_PA0_ON                             0x80  // Default
#define RF_PALEVEL_PA0_OFF                            0x00
#define RF_PALEVEL_PA1_ON                             0x40
#define RF_PALEVEL_PA1_OFF                            0x00  // Default
#define RF_PALEVEL_PA2_ON                             0x20
#define RF_PALEVEL_PA2_OFF                            0x00  // Default

// RegPaRamp
#define RF_PARAMP_3400                                0x00
#define RF_PARAMP_2000                                0x01
#define RF_PARAMP_1000                                0x02
#define RF_PARAMP_500                                 0x03
#define RF_PARAMP_250                                 0x04
#define RF_PARAMP_125                                 0x05
#define RF_PARAMP_100                                 0x06
#define RF_PARAMP_62                                  0x07
#define RF_PARAMP_50                                  0x08
#define RF_PARAMP_40                                  0x09  // Default
#define RF_PARAMP_31                                  0x0A
#define RF_PARAMP_25                                  0x0B
#define RF_PARAMP_20                                  0x0C
#define RF_PARAMP_15                                  0x0D
#define RF_PARAMP_12                                  0x0E
#define RF_PARAMP_10                                  0x0F

// RegOcp
#define RF_OCP_OFF                                    0x0F
#define RF_OCP_ON                                     0x1A  // Default

#define RF_OCP_TRIM_45                                0x00
#define RF_OCP_TRIM_50                                0x01
#define RF_OCP_TRIM_55                                0x02
#define RF_OCP_TRIM_60                                0x03
#define RF_OCP_TRIM_65                                0x04
#define RF_OCP_TRIM_70                                0x05
#define RF_OCP_TRIM_75                                0x06
#define RF_OCP_TRIM_80                                0x07
#define RF_OCP_TRIM_85                                0x08
#define RF_OCP_TRIM_90                                0x09
#define RF_OCP_TRIM_95                                0x0A
#define RF_OCP_TRIM_100                               0x0B  // Default
#define RF_OCP_TRIM_105                               0x0C
#define RF_OCP_TRIM_110                               0x0D
#define RF_OCP_TRIM_115                               0x0E
#define RF_OCP_TRIM_120                               0x0F

// RegAgcRef
#define RF_AGCREF_AUTO_ON                             0x40  // Default
#define RF_AGCREF_AUTO_OFF                            0x00

#define RF_AGCREF_LEVEL_MINUS80                       0x00  // Default
#define RF_AGCREF_LEVEL_MINUS81                       0x01
#define RF_AGCREF_LEVEL_MINUS82                       0x02
#define RF_AGCREF_LEVEL_MINUS83                       0x03
#define RF_AGCREF_LEVEL_MINUS84                       0x04
#define RF_AGCREF_LEVEL_MINUS85                       0x05
#define RF_AGCREF_LEVEL_MINUS86                       0x06
#define RF_AGCREF_LEVEL_MINUS87                       0x07
#define RF_AGCREF_LEVEL_MINUS88                       0x08
#define RF_AGCREF_LEVEL_MINUS89                       0x09
#define RF_AGCREF_LEVEL_MINUS90                       0x0A
#define RF_AGCREF_LEVEL_MINUS91                       0x0B
#define RF_AGCREF_LEVEL_MINUS92                       0x0C
#define RF_AGCREF_LEVEL_MINUS93                       0x0D
#define RF_AGCREF_LEVEL_MINUS94                       0x0E
#define RF_AGCREF_LEVEL_MINUS95                       0x0F
#define RF_AGCREF_LEVEL_MINUS96                       0x10
#define RF_AGCREF_LEVEL_MINUS97                       0x11
#define RF_AGCREF_LEVEL_MINUS98                       0x12
#define RF_AGCREF_LEVEL_MINUS99                       0x13
#define RF_AGCREF_LEVEL_MINUS100                      0x14
#define RF_AGCREF_LEVEL_MINUS101                      0x15
#define RF_AGCREF_LEVEL_MINUS102                      0x16
#define RF_AGCREF_LEVEL_MINUS103                      0x17
#define RF_AGCREF_LEVEL_MINUS104                      0x18
#define RF_AGCREF_LEVEL_MINUS105                      0x19
#define RF_AGCREF_LEVEL_MINUS106                      0x1A
#define RF_AGCREF_LEVEL_MINUS107                      0x1B
#define RF_AGCREF_LEVEL_MINUS108                      0x1C
#define RF_AGCREF_LEVEL_MINUS109                      0x1D
#define RF_AGCREF_LEVEL_MINUS110                      0x1E
#define RF_AGCREF_LEVEL_MINUS111                      0x1F
#define RF_AGCREF_LEVEL_MINUS112                      0x20
#define RF_AGCREF_LEVEL_MINUS113                      0x21
#define RF_AGCREF_LEVEL_MINUS114                      0x22
#define RF_AGCREF_LEVEL_MINUS115                      0x23
#define RF_AGCREF_LEVEL_MINUS116                      0x24
#define RF_AGCREF_LEVEL_MINUS117                      0x25
#define RF_AGCREF_LEVEL_MINUS118                      0x26
#define RF_AGCREF_LEVEL_MINUS119                      0x27
#define RF_AGCREF_LEVEL_MINUS120                      0x28
#define RF_AGCREF_LEVEL_MINUS121                      0x29
#define RF_AGCREF_LEVEL_MINUS122                      0x2A
#define RF_AGCREF_LEVEL_MINUS123                      0x2B
#define RF_AGCREF_LEVEL_MINUS124                      0x2C
#define RF_AGCREF_LEVEL_MINUS125                      0x2D
#define RF_AGCREF_LEVEL_MINUS126                      0x2E
#define RF_AGCREF_LEVEL_MINUS127                      0x2F
#define RF_AGCREF_LEVEL_MINUS128                      0x30
#define RF_AGCREF_LEVEL_MINUS129                      0x31
#define RF_AGCREF_LEVEL_MINUS130                      0x32
#define RF_AGCREF_LEVEL_MINUS131                      0x33
#define RF_AGCREF_LEVEL_MINUS132                      0x34
#define RF_AGCREF_LEVEL_MINUS133                      0x35
#define RF_AGCREF_LEVEL_MINUS134                      0x36
#define RF_AGCREF_LEVEL_MINUS135                      0x37
#define RF_AGCREF_LEVEL_MINUS136                      0x38
#define RF_AGCREF_LEVEL_MINUS137                      0x39
#define RF_AGCREF_LEVEL_MINUS138                      0x3A
#define RF_AGCREF_LEVEL_MINUS139                      0x3B
#define RF_AGCREF_LEVEL_MINUS140                      0x3C
#define RF_AGCREF_LEVEL_MINUS141                      0x3D
#define RF_AGCREF_LEVEL_MINUS142                      0x3E
#define RF_AGCREF_LEVEL_MINUS143                      0x3F

// RegAgcThresh1
#define RF_AGCTHRESH1_SNRMARGIN_000                   0x00
#define RF_AGCTHRESH1_SNRMARGIN_001                   0x20
#define RF_AGCTHRESH1_SNRMARGIN_010                   0x40
#define RF_AGCTHRESH1_SNRMARGIN_011                   0x60
#define RF_AGCTHRESH1_SNRMARGIN_100                   0x80
#define RF_AGCTHRESH1_SNRMARGIN_101                   0xA0  // Default
#define RF_AGCTHRESH1_SNRMARGIN_110                   0xC0
#define RF_AGCTHRESH1_SNRMARGIN_111                   0xE0

#define RF_AGCTHRESH1_STEP1_0                         0x00
#define RF_AGCTHRESH1_STEP1_1                         0x01
#define RF_AGCTHRESH1_STEP1_2                         0x02
#define RF_AGCTHRESH1_STEP1_3                         0x03
#define RF_AGCTHRESH1_STEP1_4                         0x04
#define RF_AGCTHRESH1_STEP1_5                         0x05
#define RF_AGCTHRESH1_STEP1_6                         0x06
#define RF_AGCTHRESH1_STEP1_7                         0x07
#define RF_AGCTHRESH1_STEP1_8                         0x08
#define RF_AGCTHRESH1_STEP1_9                         0x09
#define RF_AGCTHRESH1_STEP1_10                        0x0A
#define RF_AGCTHRESH1_STEP1_11                        0x0B
#define RF_AGCTHRESH1_STEP1_12                        0x0C
#define RF_AGCTHRESH1_STEP1_13                        0x0D
#define RF_AGCTHRESH1_STEP1_14                        0x0E
#define RF_AGCTHRESH1_STEP1_15                        0x0F
#define RF_AGCTHRESH1_STEP1_16                        0x10  // Default
#define RF_AGCTHRESH1_STEP1_17                        0x11
#define RF_AGCTHRESH1_STEP1_18                        0x12
#define RF_AGCTHRESH1_STEP1_19                        0x13
#define RF_AGCTHRESH1_STEP1_20                        0x14
#define RF_AGCTHRESH1_STEP1_21                        0x15
#define RF_AGCTHRESH1_STEP1_22                        0x16
#define RF_AGCTHRESH1_STEP1_23                        0x17
#define RF_AGCTHRESH1_STEP1_24                        0x18
#define RF_AGCTHRESH1_STEP1_25                        0x19
#define RF_AGCTHRESH1_STEP1_26                        0x1A
#define RF_AGCTHRESH1_STEP1_27                        0x1B
#define RF_AGCTHRESH1_STEP1_28                        0x1C
#define RF_AGCTHRESH1_STEP1_29                        0x1D
#define RF_AGCTHRESH1_STEP1_30                        0x1E
#define RF_AGCTHRESH1_STEP1_31                        0x1F

// RegAgcThresh2
#define RF_AGCTHRESH2_STEP2_0                         0x00
#define RF_AGCTHRESH2_STEP2_1                         0x10
#define RF_AGCTHRESH2_STEP2_2                         0x20
#define RF_AGCTHRESH2_STEP2_3                         0x30  // XXX wrong -- Default
#define RF_AGCTHRESH2_STEP2_4                         0x40
#define RF_AGCTHRESH2_STEP2_5                         0x50
#define RF_AGCTHRESH2_STEP2_6                         0x60
#define RF_AGCTHRESH2_STEP2_7                         0x70    // default
#define RF_AGCTHRESH2_STEP2_8                         0x80
#define RF_AGCTHRESH2_STEP2_9                         0x90
#define RF_AGCTHRESH2_STEP2_10                        0xA0
#define RF_AGCTHRESH2_STEP2_11                        0xB0
#define RF_AGCTHRESH2_STEP2_12                        0xC0
#define RF_AGCTHRESH2_STEP2_13                        0xD0
#define RF_AGCTHRESH2_STEP2_14                        0xE0
#define RF_AGCTHRESH2_STEP2_15                        0xF0

#define RF_AGCTHRESH2_STEP3_0                         0x00
#define RF_AGCTHRESH2_STEP3_1                         0x01
#define RF_AGCTHRESH2_STEP3_2                         0x02
#define RF_AGCTHRESH2_STEP3_3                         0x03
#define RF_AGCTHRESH2_STEP3_4                         0x04
#define RF_AGCTHRESH2_STEP3_5                         0x05
#define RF_AGCTHRESH2_STEP3_6                         0x06
#define RF_AGCTHRESH2_STEP3_7                         0x07
#define RF_AGCTHRESH2_STEP3_8                         0x08
#define RF_AGCTHRESH2_STEP3_9                         0x09
#define RF_AGCTHRESH2_STEP3_10                        0x0A
#define RF_AGCTHRESH2_STEP3_11                        0x0B  // Default
#define RF_AGCTHRESH2_STEP3_12                        0x0C
#define RF_AGCTHRESH2_STEP3_13                        0x0D
#define RF_AGCTHRESH2_STEP3_14                        0x0E
#define RF_AGCTHRESH2_STEP3_15                        0x0F

// RegAgcThresh3
#define RF_AGCTHRESH3_STEP4_0                         0x00
#define RF_AGCTHRESH3_STEP4_1                         0x10
#define RF_AGCTHRESH3_STEP4_2                         0x20
#define RF_AGCTHRESH3_STEP4_3                         0x30
#define RF_AGCTHRESH3_STEP4_4                         0x40
#define RF_AGCTHRESH3_STEP4_5                         0x50
#define RF_AGCTHRESH3_STEP4_6                         0x60
#define RF_AGCTHRESH3_STEP4_7                         0x70
#define RF_AGCTHRESH3_STEP4_8                         0x80
#define RF_AGCTHRESH3_STEP4_9                         0x90  // Default
#define RF_AGCTHRESH3_STEP4_10                        0xA0
#define RF_AGCTHRESH3_STEP4_11                        0xB0
#define RF_AGCTHRESH3_STEP4_12                        0xC0
#define RF_AGCTHRESH3_STEP4_13                        0xD0
#define RF_AGCTHRESH3_STEP4_14                        0xE0
#define RF_AGCTHRESH3_STEP4_15                        0xF0

#define RF_AGCTHRESH3_STEP5_0                         0x00
#define RF_AGCTHRESH3_STEP5_1                         0x01
#define RF_AGCTHRESH3_STEP5_2                         0x02
#define RF_AGCTHRESH3_STEP5_3                         0x03
#define RF_AGCTHRESH3_STEP5_4                         0x04
#define RF_AGCTHRESH3_STEP5_5                         0x05
#define RF_AGCTHRESH3_STEP5_6                         0x06
#define RF_AGCTHRESH3_STEP5_7                         0x07
#define RF_AGCTHRES33_STEP5_8                         0x08
#define RF_AGCTHRESH3_STEP5_9                         0x09
#define RF_AGCTHRESH3_STEP5_10                        0x0A
#define RF_AGCTHRESH3_STEP5_11                        0x0B  // Default
#define RF_AGCTHRESH3_STEP5_12                        0x0C
#define RF_AGCTHRESH3_STEP5_13                        0x0D
#define RF_AGCTHRESH3_STEP5_14                        0x0E
#define RF_AGCTHRESH3_STEP5_15                        0x0F

// RegLna
#define RF_LNA_ZIN_50                                 0x00
#define RF_LNA_ZIN_200                                0x80  // Default

#define RF_LNA_LOWPOWER_OFF                           0x00  // Default
#define RF_LNA_LOWPOWER_ON                            0x40

#define RF_LNA_CURRENTGAIN                            0x38

#define RF_LNA_GAINSELECT_AUTO                        0x00  // Default
#define RF_LNA_GAINSELECT_MAX                         0x01
#define RF_LNA_GAINSELECT_MAXMINUS6                   0x02
#define RF_LNA_GAINSELECT_MAXMINUS12                  0x03
#define RF_LNA_GAINSELECT_MAXMINUS24                  0x04
#define RF_LNA_GAINSELECT_MAXMINUS36                  0x05
#define RF_LNA_GAINSELECT_MAXMINUS48                  0x06

// RegRxBw
#define RF_RXBW_DCCFREQ_000                           0x00
#define RF_RXBW_DCCFREQ_001                           0x20
#define RF_RXBW_DCCFREQ_010                           0x40  // Default
#define RF_RXBW_DCCFREQ_011                           0x60
#define RF_RXBW_DCCFREQ_100                           0x80
#define RF_RXBW_DCCFREQ_101                           0xA0
#define RF_RXBW_DCCFREQ_110                           0xC0
#define RF_RXBW_DCCFREQ_111                           0xE0

#define RF_RXBW_MANT_16                               0x00
#define RF_RXBW_MANT_20                               0x08
#define RF_RXBW_MANT_24                               0x10  // Default

#define RF_RXBW_EXP_0                                 0x00
#define RF_RXBW_EXP_1                                 0x01
#define RF_RXBW_EXP_2                                 0x02
#define RF_RXBW_EXP_3                                 0x03
#define RF_RXBW_EXP_4                                 0x04
#define RF_RXBW_EXP_5                                 0x05  // Default
#define RF_RXBW_EXP_6                                 0x06
#define RF_RXBW_EXP_7                                 0x07

// RegAfcBw
#define RF_AFCBW_DCCFREQAFC_000                       0x00
#define RF_AFCBW_DCCFREQAFC_001                       0x20
#define RF_AFCBW_DCCFREQAFC_010                       0x40
#define RF_AFCBW_DCCFREQAFC_011                       0x60
#define RF_AFCBW_DCCFREQAFC_100                       0x80  // Default
#define RF_AFCBW_DCCFREQAFC_101                       0xA0
#define RF_AFCBW_DCCFREQAFC_110                       0xC0
#define RF_AFCBW_DCCFREQAFC_111                       0xE0

#define RF_AFCBW_MANTAFC_16                           0x00
#define RF_AFCBW_MANTAFC_20                           0x08  // Default
#define RF_AFCBW_MANTAFC_24                           0x10

#define RF_AFCBW_EXPAFC_0                             0x00
#define RF_AFCBW_EXPAFC_1                             0x01
#define RF_AFCBW_EXPAFC_2                             0x02
#define RF_AFCBW_EXPAFC_3                             0x03  // Default
#define RF_AFCBW_EXPAFC_4                             0x04
#define RF_AFCBW_EXPAFC_5                             0x05
#define RF_AFCBW_EXPAFC_6                             0x06
#define RF_AFCBW_EXPAFC_7                             0x07

// RegOokPeak
#define RF_OOKPEAK_THRESHTYPE_FIXED                   0x00
#define RF_OOKPEAK_THRESHTYPE_PEAK                    0x40  // Default
#define RF_OOKPEAK_THRESHTYPE_AVERAGE                 0x80

#define RF_OOKPEAK_PEAKTHRESHSTEP_000                 0x00  // Default
#define RF_OOKPEAK_PEAKTHRESHSTEP_001                 0x08
#define RF_OOKPEAK_PEAKTHRESHSTEP_010                 0x10
#define RF_OOKPEAK_PEAKTHRESHSTEP_011                 0x18
#define RF_OOKPEAK_PEAKTHRESHSTEP_100                 0x20
#define RF_OOKPEAK_PEAKTHRESHSTEP_101                 0x28
#define RF_OOKPEAK_PEAKTHRESHSTEP_110                 0x30
#define RF_OOKPEAK_PEAKTHRESHSTEP_111                 0x38

#define RF_OOKPEAK_PEAKTHRESHDEC_000                  0x00  // Default
#define RF_OOKPEAK_PEAKTHRESHDEC_001                  0x01
#define RF_OOKPEAK_PEAKTHRESHDEC_010                  0x02
#define RF_OOKPEAK_PEAKTHRESHDEC_011                  0x03
#define RF_OOKPEAK_PEAKTHRESHDEC_100                  0x04
#define RF_OOKPEAK_PEAKTHRESHDEC_101                  0x05
#define RF_OOKPEAK_PEAKTHRESHDEC_110                  0x06
#define RF_OOKPEAK_PEAKTHRESHDEC_111                  0x07

// RegOokAvg
#define RF_OOKAVG_AVERAGETHRESHFILT_00                0x00
#define RF_OOKAVG_AVERAGETHRESHFILT_01                0x40
#define RF_OOKAVG_AVERAGETHRESHFILT_10                0x80  // Default
#define RF_OOKAVG_AVERAGETHRESHFILT_11                0xC0

// RegOokFix
#define RF_OOKFIX_FIXEDTHRESH_VALUE                   0x06  // Default

// RegAfcFei
#define RF_AFCFEI_FEI_DONE                            0x40
#define RF_AFCFEI_FEI_START                           0x20
#define RF_AFCFEI_AFC_DONE                            0x10
#define RF_AFCFEI_AFCAUTOCLEAR_ON                     0x08
#define RF_AFCFEI_AFCAUTOCLEAR_OFF                    0x00  // Default

#define RF_AFCFEI_AFCAUTO_ON                          0x04
#define RF_AFCFEI_AFCAUTO_OFF                         0x00  // Default

#define RF_AFCFEI_AFC_CLEAR                           0x02
#define RF_AFCFEI_AFC_START                           0x01

// RegRssiConfig
#define RF_RSSI_FASTRX_ON                             0x08
#define RF_RSSI_FASTRX_OFF                            0x00  // Default
#define RF_RSSI_DONE                                  0x02
#define RF_RSSI_START                                 0x01

// RegDioMapping1
#define RF_DIOMAPPING1_DIO0_00                        0x00  // Default
#define RF_DIOMAPPING1_DIO0_01                        0x40
#define RF_DIOMAPPING1_DIO0_10                        0x80
#define RF_DIOMAPPING1_DIO0_11                        0xC0

#define RF_DIOMAPPING1_DIO1_00                        0x00  // Default
#define RF_DIOMAPPING1_DIO1_01                        0x10
#define RF_DIOMAPPING1_DIO1_10                        0x20
#define RF_DIOMAPPING1_DIO1_11                        0x30

#define RF_DIOMAPPING1_DIO2_00                        0x00  // Default
#define RF_DIOMAPPING1_DIO2_01                        0x04
#define RF_DIOMAPPING1_DIO2_10                        0x08
#define RF_DIOMAPPING1_DIO2_11                        0x0C

#define RF_DIOMAPPING1_DIO3_00                        0x00  // Default
#define RF_DIOMAPPING1_DIO3_01                        0x01
#define RF_DIOMAPPING1_DIO3_10                        0x02
#define RF_DIOMAPPING1_DIO3_11                        0x03


// RegDioMapping2
#define RF_DIOMAPPING2_DIO4_00                        0x00  // Default
#define RF_DIOMAPPING2_DIO4_01                        0x40
#define RF_DIOMAPPING2_DIO4_10                        0x80
#define RF_DIOMAPPING2_DIO4_11                        0xC0

#define RF_DIOMAPPING2_DIO5_00                        0x00  // Default
#define RF_DIOMAPPING2_DIO5_01                        0x10
#define RF_DIOMAPPING2_DIO5_10                        0x20
#define RF_DIOMAPPING2_DIO5_11                        0x30

#define RF_DIOMAPPING2_CLKOUT_32                      0x00
#define RF_DIOMAPPING2_CLKOUT_16                      0x01
#define RF_DIOMAPPING2_CLKOUT_8                       0x02
#define RF_DIOMAPPING2_CLKOUT_4                       0x03
#define RF_DIOMAPPING2_CLKOUT_2                       0x04
#define RF_DIOMAPPING2_CLKOUT_1                       0x05
#define RF_DIOMAPPING2_CLKOUT_RC                      0x06
#define RF_DIOMAPPING2_CLKOUT_OFF                     0x07  // Default

// RegIrqFlags1
#define RF_IRQFLAGS1_MODEREADY                        0x80
#define RF_IRQFLAGS1_RXREADY                          0x40
#define RF_IRQFLAGS1_TXREADY                          0x20
#define RF_IRQFLAGS1_PLLLOCK                          0x10
#define RF_IRQFLAGS1_RSSI                             0x08
#define RF_IRQFLAGS1_TIMEOUT                          0x04
#define RF_IRQFLAGS1_AUTOMODE                         0x02
#define RF_IRQFLAGS1_SYNCADDRESSMATCH                 0x01

// RegIrqFlags2
#define RF_IRQFLAGS2_FIFOFULL                         0x80
#define RF_IRQFLAGS2_FIFONOTEMPTY                     0x40
#define RF_IRQFLAGS2_FIFOLEVEL                        0x20
#define RF_IRQFLAGS2_FIFOOVERRUN                      0x10
#define RF_IRQFLAGS2_PACKETSENT                       0x08
#define RF_IRQFLAGS2_PAYLOADREADY                     0x04
#define RF_IRQFLAGS2_CRCOK                            0x02
#define RF_IRQFLAGS2_LOWBAT                           0x01

// RegRssiThresh
#define RF_RSSITHRESH_VALUE                           0xE4  // Default

// RegRxTimeout1
#define RF_RXTIMEOUT1_RXSTART_VALUE                   0x00  // Default

// RegRxTimeout2
#define RF_RXTIMEOUT2_RSSITHRESH_VALUE                0x00  // Default

// RegPreamble
#define RF_PREAMBLESIZE_MSB_VALUE                     0x00  // Default
#define RF_PREAMBLESIZE_LSB_VALUE                     0x03  // Default

// RegSyncConfig
#define RF_SYNC_ON                                    0x80  // Default
#define RF_SYNC_OFF                                   0x00

#define RF_SYNC_FIFOFILL_AUTO                         0x00  // Default -- when sync interrupt occurs
#define RF_SYNC_FIFOFILL_MANUAL                       0x40

#define RF_SYNC_SIZE_1                                0x00
#define RF_SYNC_SIZE_2                                0x08
#define RF_SYNC_SIZE_3                                0x10
#define RF_SYNC_SIZE_4                                0x18  // Default
#define RF_SYNC_SIZE_5                                0x20
#define RF_SYNC_SIZE_6                                0x28
#define RF_SYNC_SIZE_7                                0x30
#define RF_SYNC_SIZE_8                                0x38

#define RF_SYNC_TOL_0                                 0x00  // Default
#define RF_SYNC_TOL_1                                 0x01
#define RF_SYNC_TOL_2                                 0x02
#define RF_SYNC_TOL_3                                 0x03
#define RF_SYNC_TOL_4                                 0x04
#define RF_SYNC_TOL_5                                 0x05
#define RF_SYNC_TOL_6                                 0x06
#define RF_SYNC_TOL_7                                 0x07

// RegSyncValue1-8
#define RF_SYNC_BYTE1_VALUE                           0x00  // Default
#define RF_SYNC_BYTE2_VALUE                           0x00  // Default
#define RF_SYNC_BYTE3_VALUE                           0x00  // Default
#define RF_SYNC_BYTE4_VALUE                           0x00  // Default
#define RF_SYNC_BYTE5_VALUE                           0x00  // Default
#define RF_SYNC_BYTE6_VALUE                           0x00  // Default
#define RF_SYNC_BYTE7_VALUE                           0x00  // Default
#define RF_SYNC_BYTE8_VALUE                           0x00  // Default

// RegPacketConfig1
#define RF_PACKET1_FORMAT_FIXED                       0x00  // Default
#define RF_PACKET1_FORMAT_VARIABLE                    0x80

#define RF_PACKET1_DCFREE_OFF                         0x00  // Default
#define RF_PACKET1_DCFREE_MANCHESTER                  0x20
#define RF_PACKET1_DCFREE_WHITENING                   0x40

#define RF_PACKET1_CRC_ON                             0x10  // Default
#define RF_PACKET1_CRC_OFF                            0x00

#define RF_PACKET1_CRCAUTOCLEAR_ON                    0x00  // Default
#define RF_PACKET1_CRCAUTOCLEAR_OFF                   0x08

#define RF_PACKET1_ADRSFILTERING_OFF                  0x00  // Default
#define RF_PACKET1_ADRSFILTERING_NODE                 0x02
#define RF_PACKET1_ADRSFILTERING_NODEBROADCAST        0x04

// RegPayloadLength
#define RF_PAYLOADLENGTH_VALUE                        0x40  // Default

// RegBroadcastAdrs
#define RF_BROADCASTADDRESS_VALUE                     0x00

// RegAutoModes
#define RF_AUTOMODES_ENTER_OFF                        0x00  // Default
#define RF_AUTOMODES_ENTER_FIFONOTEMPTY               0x20
#define RF_AUTOMODES_ENTER_FIFOLEVEL                  0x40
#define RF_AUTOMODES_ENTER_CRCOK                      0x60
#define RF_AUTOMODES_ENTER_PAYLOADREADY               0x80
#define RF_AUTOMODES_ENTER_SYNCADRSMATCH              0xA0
#define RF_AUTOMODES_ENTER_PACKETSENT                 0xC0
#define RF_AUTOMODES_ENTER_FIFOEMPTY                  0xE0

#define RF_AUTOMODES_EXIT_OFF                         0x00  // Default
#define RF_AUTOMODES_EXIT_FIFOEMPTY                   0x04
#define RF_AUTOMODES_EXIT_FIFOLEVEL                   0x08
#define RF_AUTOMODES_EXIT_CRCOK                       0x0C
#define RF_AUTOMODES_EXIT_PAYLOADREADY                0x10
#define RF_AUTOMODES_EXIT_SYNCADRSMATCH               0x14
#define RF_AUTOMODES_EXIT_PACKETSENT                  0x18
#define RF_AUTOMODES_EXIT_RXTIMEOUT                   0x1C

#define RF_AUTOMODES_INTERMEDIATE_SLEEP               0x00  // Default
#define RF_AUTOMODES_INTERMEDIATE_STANDBY             0x01
#define RF_AUTOMODES_INTERMEDIATE_RECEIVER            0x02
#define RF_AUTOMODES_INTERMEDIATE_TRANSMITTER         0x03

// RegFifoThresh
#define RF_FIFOTHRESH_TXSTART_FIFOTHRESH              0x00
#define RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY            0x80  // Default

#define RF_FIFOTHRESH_VALUE                           0x0F  // Default

// RegPacketConfig2
#define RF_PACKET2_RXRESTARTDELAY_1BIT                0x00  // Default
#define RF_PACKET2_RXRESTARTDELAY_2BITS               0x10
#define RF_PACKET2_RXRESTARTDELAY_4BITS               0x20
#define RF_PACKET2_RXRESTARTDELAY_8BITS               0x30
#define RF_PACKET2_RXRESTARTDELAY_16BITS              0x40
#define RF_PACKET2_RXRESTARTDELAY_32BITS              0x50
#define RF_PACKET2_RXRESTARTDELAY_64BITS              0x60
#define RF_PACKET2_RXRESTARTDELAY_128BITS             0x70
#define RF_PACKET2_RXRESTARTDELAY_256BITS             0x80
#define RF_PACKET2_RXRESTARTDELAY_512BITS             0x90
#define RF_PACKET2_RXRESTARTDELAY_1024BITS            0xA0
#define RF_PACKET2_RXRESTARTDELAY_2048BITS            0xB0
#define RF_PACKET2_RXRESTARTDELAY_NONE                0xC0
#define RF_PACKET2_RXRESTART                          0x04

#define RF_PACKET2_AUTORXRESTART_ON                   0x02  // Default
#define RF_PACKET2_AUTORXRESTART_OFF                  0x00

#define RF_PACKET2_AES_ON                             0x01
#define RF_PACKET2_AES_OFF                            0x00  // Default

// RegAesKey1-16
#define RF_AESKEY1_VALUE                              0x00  // Default
#define RF_AESKEY2_VALUE                              0x00  // Default
#define RF_AESKEY3_VALUE                              0x00  // Default
#define RF_AESKEY4_VALUE                              0x00  // Default
#define RF_AESKEY5_VALUE                              0x00  // Default
#define RF_AESKEY6_VALUE                              0x00  // Default
#define RF_AESKEY7_VALUE                              0x00  // Default
#define RF_AESKEY8_VALUE                              0x00  // Default
#define RF_AESKEY9_VALUE                              0x00  // Default
#define RF_AESKEY10_VALUE                             0x00  // Default
#define RF_AESKEY11_VALUE                             0x00  // Default
#define RF_AESKEY12_VALUE                             0x00  // Default
#define RF_AESKEY13_VALUE                             0x00  // Default
#define RF_AESKEY14_VALUE                             0x00  // Default
#define RF_AESKEY15_VALUE                             0x00  // Default
#define RF_AESKEY16_VALUE                             0x00  // Default

// RegTemp1
#define RF_TEMP1_MEAS_START                           0x08
#define RF_TEMP1_MEAS_RUNNING                         0x04
#define RF_TEMP1_ADCLOWPOWER_ON                       0x01  // Default
#define RF_TEMP1_ADCLOWPOWER_OFF                      0x00

// RegTestPa1
#define RF_PA20DBM1_NORMAL_AND_RX                     0x55  // Default
#define RF_PA20DBM1_20_DBM_MODE                       0x5D

// RegTestPa2
#define RF_PA20DBM2_NORMAL_AND_RX                     0x70  // Default
#define RF_PA20DBM2_20_DBM_MODE                       0x7C

// RegTestDagc
#define RF_DAGC_NORMAL                                0x00  // Reset value
#define RF_DAGC_IMPROVED_LOWBETA1                     0x20
#define RF_DAGC_IMPROVED_LOWBETA0                     0x30  // Recommended default

// RegBitRate (bits/sec) example bit rates
#define RF_BITRATEMSB_1200                            0x68
#define RF_BITRATELSB_1200                            0x2B
#define RF_BITRATEMSB_2400                            0x34
#define RF_BITRATELSB_2400                            0x15
#define RF_BITRATEMSB_4800                            0x1A  // Default
#define RF_BITRATELSB_4800                            0x0B  // Default
#define RF_BITRATEMSB_9600                            0x0D
#define RF_BITRATELSB_9600                            0x05
#define RF_BITRATEMSB_19200                           0x06
#define RF_BITRATELSB_19200                           0x83
#define RF_BITRATEMSB_38400                           0x03
#define RF_BITRATELSB_38400                           0x41
#define RF_BITRATEMSB_38323                           0x03
#define RF_BITRATELSB_38323                           0x43
#define RF_BITRATEMSB_34482                           0x03
#define RF_BITRATELSB_34482                           0xA0
#define RF_BITRATEMSB_76800                           0x01
#define RF_BITRATELSB_76800                           0xA1
#define RF_BITRATEMSB_153600                          0x00
#define RF_BITRATELSB_153600                          0xD0
#define RF_BITRATEMSB_57600                           0x02
#define RF_BITRATELSB_57600                           0x2C
#define RF_BITRATEMSB_115200                          0x01
#define RF_BITRATELSB_115200                          0x16
#define RF_BITRATEMSB_12500                           0x0A
#define RF_BITRATELSB_12500                           0x00
#define RF_BITRATEMSB_25000                           0x05
#define RF_BITRATELSB_25000                           0x00
#define RF_BITRATEMSB_50000                           0x02
#define RF_BITRATELSB_50000                           0x80
#define RF_BITRATEMSB_100000                          0x01
#define RF_BITRATELSB_100000                          0x40
#define RF_BITRATEMSB_150000                          0x00
#define RF_BITRATELSB_150000                          0xD5
#define RF_BITRATEMSB_200000                          0x00
#define RF_BITRATELSB_200000                          0xA0
#define RF_BITRATEMSB_250000                          0x00
#define RF_BITRATELSB_250000                          0x80
#define RF_BITRATEMSB_300000                          0x00
#define RF_BITRATELSB_300000                          0x6B
#define RF_BITRATEMSB_32768                           0x03
#define RF_BITRATELSB_32768                           0xD1
#define RF_BITRATEMSB_17241                           0x07	// WH24 Weather Station
#define RF_BITRATELSB_17241                           0x40	// WH24 Weather Station

// RegFdev - frequency deviation (Hz)
#define RF_FDEVMSB_2000                               0x00
#define RF_FDEVLSB_2000                               0x21
#define RF_FDEVMSB_5000                               0x00  // Default
#define RF_FDEVLSB_5000                               0x52  // Default
#define RF_FDEVMSB_7500                               0x00
#define RF_FDEVLSB_7500                               0x7B
#define RF_FDEVMSB_10000                              0x00
#define RF_FDEVLSB_10000                              0xA4
#define RF_FDEVMSB_15000                              0x00
#define RF_FDEVLSB_15000                              0xF6
#define RF_FDEVMSB_20000                              0x01
#define RF_FDEVLSB_20000                              0x48
#define RF_FDEVMSB_25000                              0x01
#define RF_FDEVLSB_25000                              0x9A
#define RF_FDEVMSB_30000                              0x01
#define RF_FDEVLSB_30000                              0xEC
#define RF_FDEVMSB_35000                              0x02
#define RF_FDEVLSB_35000                              0x3D
#define RF_FDEVMSB_40000                              0x02
#define RF_FDEVLSB_40000                              0x8F
#define RF_FDEVMSB_45000                              0x02
#define RF_FDEVLSB_45000                              0xE1
#define RF_FDEVMSB_50000                              0x03
#define RF_FDEVLSB_50000                              0x33
#define RF_FDEVMSB_55000                              0x03
#define RF_FDEVLSB_55000                              0x85
#define RF_FDEVMSB_60000                              0x03
#define RF_FDEVLSB_60000                              0xD7
#define RF_FDEVMSB_65000                              0x04
#define RF_FDEVLSB_65000                              0x29
#define RF_FDEVMSB_70000                              0x04
#define RF_FDEVLSB_70000                              0x7B
#define RF_FDEVMSB_75000                              0x04
#define RF_FDEVLSB_75000                              0xCD
#define RF_FDEVMSB_80000                              0x05
#define RF_FDEVLSB_80000                              0x1F
#define RF_FDEVMSB_85000                              0x05
#define RF_FDEVLSB_85000                              0x71
#define RF_FDEVMSB_90000                              0x05
#define RF_FDEVLSB_90000                              0xC3
#define RF_FDEVMSB_95000                              0x06
#define RF_FDEVLSB_95000                              0x14
#define RF_FDEVMSB_100000                             0x06
#define RF_FDEVLSB_100000                             0x66
#define RF_FDEVMSB_110000                             0x07
#define RF_FDEVLSB_110000                             0x0A
#define RF_FDEVMSB_120000                             0x07
#define RF_FDEVLSB_120000                             0xAE
#define RF_FDEVMSB_130000                             0x08
#define RF_FDEVLSB_130000                             0x52
#define RF_FDEVMSB_140000                             0x08
#define RF_FDEVLSB_140000                             0xF6
#define RF_FDEVMSB_150000                             0x09
#define RF_FDEVLSB_150000                             0x9A
#define RF_FDEVMSB_160000                             0x0A
#define RF_FDEVLSB_160000                             0x3D
#define RF_FDEVMSB_170000                             0x0A
#define RF_FDEVLSB_170000                             0xE1
#define RF_FDEVMSB_180000                             0x0B
#define RF_FDEVLSB_180000                             0x85
#define RF_FDEVMSB_190000                             0x0C
#define RF_FDEVLSB_190000                             0x29
#define RF_FDEVMSB_200000                             0x0C
#define RF_FDEVLSB_200000                             0xCD
#define RF_FDEVMSB_210000                             0x0D
#define RF_FDEVLSB_210000                             0x71
#define RF_FDEVMSB_220000                             0x0E
#define RF_FDEVLSB_220000                             0x14
#define RF_FDEVMSB_230000                             0x0E
#define RF_FDEVLSB_230000                             0xB8
#define RF_FDEVMSB_240000                             0x0F
#define RF_FDEVLSB_240000                             0x5C
#define RF_FDEVMSB_250000                             0x10
#define RF_FDEVLSB_250000                             0x00
#define RF_FDEVMSB_260000                             0x10
#define RF_FDEVLSB_260000                             0xA4
#define RF_FDEVMSB_270000                             0x11
#define RF_FDEVLSB_270000                             0x48
#define RF_FDEVMSB_280000                             0x11
#define RF_FDEVLSB_280000                             0xEC
#define RF_FDEVMSB_290000                             0x12
#define RF_FDEVLSB_290000                             0x8F
#define RF_FDEVMSB_300000                             0x13
#define RF_FDEVLSB_300000                             0x33
#define PAYLOAD_SIZE                17

		WriteReg(REG_OPMODE, RF_OPMODE_STANDBY |
					 RF_OPMODE_SEQUENCER_ON |
					 RF_OPMODE_LISTEN_OFF);

		WriteReg(REG_DATAMODUL, RFMODUL_MODULATIONSHAPING_00 |
					 RFMODUL_MODULATIONTYPE_FSK |
					 RFMODULMODE_PACKET);

		WriteReg(REG_BITRATEMSB, RF_BITRATEMSB_17241);	//WH24 Bit Rate
		WriteReg(REG_BITRATELSB, RF_BITRATELSB_17241);
		WriteReg(REG_FDEVMSB, RF_FDEVMSB_60000);
		WriteReg(REG_FDEVLSB, RF_FDEVLSB_60000);
#if 0
		WriteReg(REG_FRFMSB, 0x6C);      // 433.92 MHz for WH24
		WriteReg(REG_FRFMID, 0x7A);
		WriteReg(REG_FRFLSB, 0xE1);
#else
		  SetFrequency(m_frequency);
#endif
		//WriteReg(REG_OSC1, 0x00);
		WriteReg(REG_AFCCTRL, 0x00);
		//WriteReg(REG_LISTEN1, 0x00);
		//WriteReg(REG_LISTEN2, 0x00);
		//WriteReg(REG_LISTEN3, 0x00);
		//WriteReg(REG_VERSION, 0x00);
		WriteReg(REG_PALEVEL, 0x1F |
					 RF_PALEVEL_PA0_OFF |
					 RF_PALEVEL_PA1_ON |
					 RF_PALEVEL_PA2_ON);
		WriteReg(REG_PARAMP, RF_PARAMP_40);
		WriteReg(REG_OCP,  RF_OCP_OFF |
					 RF_OCP_TRIM_120);
		WriteReg(RFM69_REG_18_LNA,  RF_LNA_ZIN_50 |
					 RF_LNA_GAINSELECT_AUTO);

		WriteReg(RFM69_REG_19_RX_BW,  RF_RXBW_DCCFREQ_010 |
					 RF_RXBW_MANT_16 |
					 RF_RXBW_EXP_0);
		WriteReg(RFM69_REG_1A_AFC_BW, RF_AFCBW_DCCFREQAFC_100 |
					 RF_AFCBW_MANTAFC_20 |
					 RF_AFCBW_EXPAFC_3);

		//WriteReg(RFM69_REG_1B_OOK_PEAK,
		//WriteReg(RFM69_REG_1C_OOK_AVG,
		//WriteReg(RFM69_REG_1D_OOF_FIX,
		WriteReg(RFM69_REG_1E_AFC_FEI,  RF_AFCFEI_AFCAUTOCLEAR_OFF |
					 RF_AFCFEI_AFCAUTO_OFF);
		//WriteReg(RFM69_REG_1F_AFC_MSB,
		//WriteReg(RFM69_REG_20_AFC_LSB,
		//WriteReg(RFM69_REG_21_FEI_MSB,
		//WriteReg(RFM69_REG_22_FEI_LSB,
		//WriteReg(RFM69_REG_23_RSSI_CONFIG,
		//WriteReg(RFM69_REG_24_RSSI_VALUE,

		WriteReg(RFM69_REG_25_DIO_MAPPING1, RF_DIOMAPPING1_DIO0_01 |
					 RF_DIOMAPPING1_DIO1_00 |
					 RF_DIOMAPPING1_DIO2_00 |
					 RF_DIOMAPPING1_DIO3_00);
		WriteReg(RFM69_REG_26_DIO_MAPPING2, RF_DIOMAPPING2_DIO4_00 |
					 RF_DIOMAPPING2_DIO5_00 |
					 RF_DIOMAPPING2_CLKOUT_OFF);

		//WriteReg(RFM69_REG_27_IRQ_FLAGS1,
		//WriteReg(RFM69_REG_28_IRQ_FLAGS2,
		WriteReg(RFM69_REG_29_RSSI_THRESHOLD, RF_RSSITHRESH_VALUE);
		WriteReg(RFM69_REG_2A_RX_TIMEOUT1,  RF_RXTIMEOUT1_RXSTART_VALUE);
		WriteReg(RFM69_REG_2B_RX_TIMEOUT2,  RF_RXTIMEOUT2_RSSITHRESH_VALUE);
		WriteReg(RFM69_REG_2C_PREAMBLE_MSB, 0x00);  // 0x00
		WriteReg(RFM69_REG_2D_PREAMBLE_LSB, 0x05);  // 0x05 x 0xAA

		WriteReg(RFM69_REG_2E_SYNC_CONFIG,  RF_SYNC_ON |
					 RF_SYNC_FIFOFILL_AUTO |
					 RF_SYNC_SIZE_2 |
					 RF_SYNC_TOL_0);

		WriteReg(RFM69_REG_2F_SYNCVALUE1, 0x2D);  // 0x2D
		WriteReg(RFM69_REG_30_SYNCVALUE2, 0xD4);  // 0xD4
		WriteReg(RFM69_REG_31_SYNCVALUE3, 0x55);
		WriteReg(RFM69_REG_32_SYNCVALUE4, 0x55);
		WriteReg(RFM69_REG_33_SYNCVALUE5, 0x55);
		WriteReg(RFM69_REG_34_SYNCVALUE6, 0x55);
		WriteReg(RFM69_REG_35_SYNCVALUE7, 0x55);
		WriteReg(RFM69_REG_36_SYNCVALUE8, 0x55);

		WriteReg(RFM69_REG_37_PACKET_CONFIG1, RF_PACKET1_FORMAT_FIXED |
					 RF_PACKET1_DCFREE_OFF |
					 RF_PACKET1_CRC_OFF |
					 RF_PACKET1_CRCAUTOCLEAR_OFF |
					 RF_PACKET1_ADRSFILTERING_NODE);
		WriteReg(RFM69_REG_38_PAYLOAD_LENGTH, PAYLOAD_SIZE);

		//WriteReg(RFM69_REG_39_NODE_ADDRESS,
		WriteReg(RFM69_REG_39_NODE_ADDRESS, 0x24);

		//WriteReg(RFM69_REG_3A_BROADCAST_ADDRESS,
		WriteReg(RFM69_REG_3B_AUTOMODES,  RF_AUTOMODES_ENTER_OFF |
					 RF_AUTOMODES_EXIT_OFF |
					 RF_AUTOMODES_INTERMEDIATE_SLEEP);
		WriteReg(RFM69_REG_3C_FIFO_THRESHOLD, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY |
					 RF_FIFOTHRESH_VALUE);
		WriteReg(RFM69_REG_3D_PACKET_CONFIG2, RF_PACKET2_RXRESTARTDELAY_1BIT |
					 //RF_PACKET2_AUTORXRESTART_ON |
					 RF_PACKET2_AES_OFF);
		//WriteReg(RFM69_REG_3E_AES_KEY_BYTE1, RF_AESKEY1_VALUE);
		//...
		//WriteReg(RFM69_REG_3F_AES_KEY_BYTE2, RF_AESKEY2_VALUE);
		//WriteReg(RFM69_REG_58_TEST_LNA,
		WriteReg(RFM69_REG_5A_TEST_PA1, RF_PA20DBM1_NORMAL_AND_RX);
		WriteReg(RFM69_REG_5C_TEST_PA2, RF_PA20DBM2_NORMAL_AND_RX);
		WriteReg(RFM69_REG_6F_TEST_DAGC, RF_DAGC_IMPROVED_LOWBETA0);
		//WriteReg(RFM69_REG_71_TEST_AFC,
#endif
  }
  else {
    spi16(0x8208);              // RX/TX off
    spi16(0x80E8);              // 80e8 CONFIGURATION EL,EF,868 band,12.5pF  (iT+ 915  80f8)
    spi16(0xC26a);              // DATA FILTER
    spi16(0xCA12);              // FIFO AND RESET  8,SYNC,!ff,DR
    spi16(0xCEd4);              // SYNCHRON PATTERN  0x2dd4
    spi16(0xC481);              // AFC during VDI HIGH
    spi16(0x94a0);              // RECEIVER CONTROL VDI Medium 134khz LNA max DRRSI 103 dbm
    spi16(0xCC77);              //
    spi16(0x9850);              // Deviation 90 kHz
    spi16(0xE000);              //
    spi16(0xC800);              //
    spi16(0xC040);              // 1.66MHz,2.2V
  }
#if 0
  SetFrequency(m_frequency);
  SetDataRate(m_dataRate);
#endif
  ClearFifo();
}

#if 1
byte RFM::spi8(byte value) {
  byte res;
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));
  digitalWrite(m_ss, LOW);
  res = SPI.transfer(value);
  digitalWrite(m_ss, HIGH);
  SPI.endTransaction();
  return res;
}

unsigned short RFM::spi16(unsigned short value) {
  unsigned short res;
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));
  digitalWrite(m_ss, LOW);
  res = SPI.transfer16(value);
  digitalWrite(m_ss, HIGH);
  SPI.endTransaction();
  return res;
}

byte RFM::ReadReg(byte addr) {
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));
  digitalWrite(m_ss, LOW);
  SPI.transfer(addr & 0x7F);
  uint8_t regval = SPI.transfer(0);
  digitalWrite(m_ss, HIGH);
  SPI.endTransaction();
  return regval;
}

void RFM::WriteReg(byte addr, byte value) {
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));
  digitalWrite(m_ss, LOW);
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  digitalWrite(m_ss, HIGH);
  SPI.endTransaction();
}
#else
#define clrb(pin) (*portOutputRegister(digitalPinToPort(pin)) &= ~digitalPinToBitMask(pin))
#define setb(pin) (*portOutputRegister(digitalPinToPort(pin)) |= digitalPinToBitMask(pin))
byte RFM::spi8(byte value) {
  volatile byte *misoPort = portInputRegister(digitalPinToPort(m_miso));
  byte misoBit = digitalPinToBitMask(m_miso);
  for (byte i = 8; i; i--) {
    clrb(m_sck);
    if (value & 0x80) {
      setb(m_mosi);
    }
    else {
      clrb(m_mosi);
    }
    value <<= 1;
    setb(m_sck);
    if (*misoPort & misoBit) {
      value |= 1;
    }
  }
  clrb(m_sck);

  return value;
}

unsigned short RFM::spi16(unsigned short value) {
  volatile byte *misoPort = portInputRegister(digitalPinToPort(m_miso));
  byte misoBit = digitalPinToBitMask(m_miso);

  clrb(m_ss);
  for (byte i = 0; i < 16; i++) {
    if (value & 32768) {
      setb(m_mosi);
    }
    else {
      clrb(m_mosi);
    }
    value <<= 1;
    if (*misoPort & misoBit) {
      value |= 1;
    }
    setb(m_sck);
    asm("nop");
    asm("nop");
    clrb(m_sck);
  }
  setb(m_ss);
  return value;
}

byte RFM::ReadReg(byte addr) {
  digitalWrite(m_ss, LOW);
  spi8(addr & 0x7F);
  byte regval = spi8(0);
  digitalWrite(m_ss, HIGH);

if (regval != 0) {
	Serial.print("ReadReg:");
	Serial.println(addr);
	Serial.println(regval);
}

  return regval;

}

void RFM::WriteReg(byte addr, byte value) {
  digitalWrite(m_ss, LOW);
  spi8(addr | 0x80);
  spi8(value);

  digitalWrite(m_ss, HIGH);
}
#endif

RFM::RadioType RFM::GetRadioType() {
  return m_radioType;
}

String RFM::GetRadioName() {
  switch (GetRadioType()) {
    case RFM::RFM12B:
      return String("RFM12B");
      break;
    case RFM::RFM69CW:
      return String("RFM69CW");
      break;
    default:
      return String("None");
  }
}

bool RFM::IsConnected() {
  return m_radioType != RFM::None;
}

void RFM::Begin(bool isPrimary) {
  // No radio found until now
  m_radioType = RFM::None;

  // Is there a RFM69 ?
  WriteReg(REG_PAYLOADLENGTH, 0xA);
  if (ReadReg(REG_PAYLOADLENGTH) == 0xA) {
    WriteReg(REG_PAYLOADLENGTH, 0x40);
    if (ReadReg(REG_PAYLOADLENGTH) == 0x40) {
      m_radioType = RFM::RFM69CW;
    }
  }
#if 1
  if (isPrimary) {
  	m_radioType = RFM::RFM69CW;
  }
  else {
	  m_radioType = RFM::None;
	}
	return;
#endif
  // Is there a RFM12 ?
  if (m_radioType == RFM::None) {
    if (isPrimary) {
      m_radioType = RFM::RFM12B;
    }
    else {
      spi16(0x820C); // Osc. + LBD
      for (int i = 0; i < 1000; i++) {
        asm("nop");
      }

      spi16(0xC04F); // LBD=3.7V
      for (int i = 0; i < 1000; i++) {
        asm("nop");
      }
      if ((spi16(0x0000) & 0x0400) == 0x0400) {
        spi16(0xC040);  // LBD = 2.2V
        for (int i = 0; i < 1000; i++) {
          asm("nop");
        }

        if ((spi16(0x0000) & 0x0400) == 0) {
          m_radioType = RFM::RFM12B;
        }
      }
    }
  }
}

RFM::RFM(byte mosi, byte miso, byte sck, byte ss, byte irq) {
  m_mosi = mosi;
  m_miso = miso;
  m_sck = sck;
  m_ss = ss;
  m_irq = irq;

  m_debug = false;
  m_dataRate = 17241;
  m_frequency = 868300;
  m_payloadPointer = 0;
  m_lastReceiveTime = 0;
  m_payloadReady = false;


  pinMode(m_mosi, OUTPUT);
  pinMode(m_miso, INPUT);
  pinMode(m_sck, OUTPUT);
  pinMode(m_ss, OUTPUT);
//  pinMode(m_irq, INPUT);
//  delay(10);
//	  SPI.begin();

  digitalWrite(m_ss, HIGH);

}

void RFM::SetDebugMode(boolean mode) {
  m_debug = mode;
}
unsigned long RFM::GetDataRate() {
  return m_dataRate;
}

unsigned long RFM::GetFrequency() {
  return m_frequency;
}
void RFM::SendByte(byte data) {
  while (!(spi16(0x0000) & 0x8000)) {}
  RFM::spi16(0xB800 | data);
}


void RFM::SendArray(byte *data, byte length) {
  if (IsRF69) {
    WriteReg(REG_PACKETCONFIG2, (ReadReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks

    EnableReceiver(false);
    ClearFifo();

    noInterrupts();
    digitalWrite(m_ss, LOW);

    spi8(REG_FIFO | 0x80);
    for (byte i = 0; i < length; i++) {
      spi8(data[i]);
    }

    digitalWrite(m_ss, HIGH);
    interrupts();

    EnableTransmitter(true);

    // Wait until transmission is finished
    unsigned long txStart = millis();
    while (!(ReadReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) && millis() - txStart < 500);

    EnableTransmitter(false);
  }
  else {
    // Transmitter on
    EnableTransmitter(true);

    // Sync, sync, sync ...
    RFM::SendByte(0xAA);
    RFM::SendByte(0xAA);
    RFM::SendByte(0xAA);
    RFM::SendByte(0x2D);
    RFM::SendByte(0xD4);

    // Send the data
    for (int i = 0; i < length; i++) {
      RFM::SendByte(data[i]);
    }

    // Transmitter off
    delay(1);
    EnableTransmitter(false);
  }

  if (m_debug) {
    Serial.print("Sending data: ");
    for (int p = 0; p < length; p++) {
      Serial.print(data[p], DEC);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void RFM::SetHFParameter(byte address, byte value) {
  WriteReg(address, value);
  if (m_debug) {
    Serial.print("WriteReg:");
    Serial.print(address);
    Serial.print("->");
    Serial.print(value);
  }
}

void RFM::SetHFParameter(unsigned short value) {
  spi16(value);
  if (m_debug) {
    Serial.print("spi16:");
    Serial.print(value);
  }
}
