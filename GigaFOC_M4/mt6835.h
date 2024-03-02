#pragma once
#include <SimpleFOC.h>

class SensorMT6835 : public Sensor {
 public:
  SensorMT6835(int cs = -1, SPISettings settings = SPISettings(1000000, MSBFIRST, SPI_MODE3)) :
    m_angle(0),
    m_spi(0),
    m_settings(settings),
    m_cs(cs),
    m_underVoltage(false),
    m_weakField(false),
    m_rotationOverspeed(false) {}

  void init(SPIClass* spi = &SPI) {
    m_spi = spi;
    if (m_cs >= 0) {
      pinMode(m_cs, OUTPUT);
      digitalWrite(m_cs, HIGH);
    }
    Sensor::init();
  }

  enum Cmd {
    kRead = 3,
    kWrite = 6,
    kProgramEEPROM = 12,
    kAutoSettingZeroPosition = 5,
    kBurstAngleRead = 10
  };

  enum Reg {
    kRegAngle2 = 0x03,
    kRegAngle1 = 0x04,
    kRegAngle0 = 0x05,
    kRegAngleCRC = 0x06,
  };

  bool underVoltage()      { return m_underVoltage;      };
  bool weakField()         { return m_weakField;         };
  bool rotationOverspeed() { return m_rotationOverspeed; };

  float getSensorAngle() override {
    uint32_t reg = readAngleReg();
    m_underVoltage = reg & 0x1;
    m_weakField = reg & 0x2;
    m_rotationOverspeed = reg & 0x4;
    m_angle = (reg >> 3) & 0x1fffffUL;
    return ((float)(m_angle) / (float)(0x200000)) * _2PI;
  }

  uint8_t transfer(Cmd cmd, uint16_t addr, uint8_t data) {
    uint8_t in[3] = {};
    uint8_t out[3] = {};
    out[0] = (uint8_t)(cmd << 4) + (uint8_t)(addr >> 8);
    out[1] = addr & 0xff;
    out[2] = data;
    if (m_cs >= 0)
      digitalWrite(m_cs, LOW);
    m_spi->beginTransaction(m_settings);
    for (int i = 0; i < sizeof(in); i++)
      in[i] = m_spi->transfer(out[i]);
    m_spi->endTransaction();
    if (m_cs >= 0)
      digitalWrite(m_cs, HIGH);
    return in[2];
  }

  /*uint32_t readAngleReg() {
    uint8_t in[6] = {};
    uint8_t out[6] = {};
    out[0] = (uint8_t)(kBurstAngleRead << 4) + (uint8_t)(kRegAngle2 >> 8);
    out[1] = kRegAngle2 & 0xff;
    if (m_cs >= 0)
      digitalWrite(m_cs, LOW);
    m_spi->beginTransaction(m_settings);
    for (int i = 0; i < sizeof(in); i++)
      in[i] = m_spi->transfer(out[i]);
    m_spi->endTransaction();
    if (m_cs >= 0)
      digitalWrite(m_cs, HIGH);
    return (in[2] << 16) | (in[3] << 8) | in[4];
  }*/
  
  uint32_t readAngleReg() {
    uint8_t data[6] = {};
    data[0] = (uint8_t)(kBurstAngleRead << 4) + (uint8_t)(kRegAngle2 >> 8);
    data[1] = kRegAngle2 & 0xff;
    if (m_cs >= 0)
      digitalWrite(m_cs, LOW);
    m_spi->beginTransaction(m_settings);
    m_spi->transfer(data, sizeof(data));
    m_spi->endTransaction();
    if (m_cs >= 0)
      digitalWrite(m_cs, HIGH);
    return (data[2] << 16) | (data[3] << 8) | data[4];
  }


 private:
  uint32_t m_angle;

  SPIClass *m_spi;
  SPISettings m_settings;
  int8_t    m_cs;

  bool m_underVoltage;
  bool m_weakField;
  bool m_rotationOverspeed;
};
