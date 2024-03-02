#pragma once
#include <SimpleFOC.h>

class SensorMT6701SSI : public Sensor {
 public:
  SensorMT6701SSI(int nCS = -1, SPISettings settings = SPISettings(1000000, MSBFIRST, SPI_MODE2)) :
      m_settings(settings), m_nCS(nCS) {}
  virtual ~SensorMT6701SSI() = default;
  virtual void init(SPIClass* spi = &SPI) {
    m_spi = spi;
    if (m_nCS >= 0) {
      pinMode(m_nCS, OUTPUT);
      digitalWrite(m_nCS, HIGH);
    }
    Sensor::init();
  }

  float getSensorAngle() override {
    float angle_data = readRawAngleSSI();
    angle_data = (angle_data / (float)0x4000) * _2PI;
    return angle_data;
  }

protected:
  uint16_t readRawAngleSSI() {
    if (m_nCS >= 0)
      digitalWrite(m_nCS, LOW);
    m_spi->beginTransaction(m_settings);
    uint16_t value = m_spi->transfer16(0x1000);
    m_spi->endTransaction();
    if (m_nCS >= 0)
      digitalWrite(m_nCS, HIGH);
    return ((value >> 0) & 0x3FFF);
  };

  SPISettings m_settings;
  SPIClass* m_spi;
  int m_nCS = -1;
};
