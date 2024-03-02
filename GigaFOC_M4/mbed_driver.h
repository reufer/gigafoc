#pragma once

#include <drivers/hardware_api.h>

/**
 3 PWM BLDC driver class

https://github.com/arduino/ArduinoCore-mbed/blob/main/cores/arduino/wiring_analog.cpp
https://github.com/arduino/mbed-os/blob/master/drivers/include/drivers/PwmOut.h
https://github.com/arduino/mbed-os/blob/master/drivers/source/PwmOut.cpp
https://github.com/arduino/mbed-os/blob/master/targets/TARGET_STM/pwmout_api.c
https://github.com/STMicroelectronics/stm32h7xx_hal_driver/blob/master/Src/stm32h7xx_hal_tim.c

*/

class MbedDriver3PWM: public BLDCDriver
{
 public:

  MbedDriver3PWM(int phA,int phB,int phC, int en1 = NOT_SET, int en2 = NOT_SET, int en3 = NOT_SET)
    : m_pwm{ 0, 0, 0 }, m_pwmPin{ phA, phB, phC }, m_pwmEnablePin{ en1, en2, en3 }, enable_active_high(true) {
    voltage_power_supply = DEF_POWER_SUPPLY;
    voltage_limit = NOT_SET;
    pwm_frequency = NOT_SET;
  }
    
  int init() override {
    for (int i = 0; i < 3; i++) {
      pinMode(m_pwmPin[i], OUTPUT);
      if (_isset(m_pwmEnablePin[i]))
        pinMode(m_pwmEnablePin[i], OUTPUT);

      m_pwm[i] = new mbed::PwmOut(digitalPinToPinName(m_pwmPin[i]));
      m_pwm[i]->period_us(40); // 25kHz
      m_pwm[i]->write(0.0f);
    }

    // sanity check for the voltage limit configuration
    if (!_isset(voltage_limit) || voltage_limit > voltage_power_supply)
      voltage_limit = voltage_power_supply;

    return 0;
  }

  void disable() override {
    setPwm(0, 0, 0);
    // disable the driver - if enable_pin pin available
    for (int i = 0; i < 3; i++)
      if (_isset(m_pwmEnablePin[i])) digitalWrite(m_pwmEnablePin[i], !enable_active_high);
  }

  void enable() override {
    for (int i = 0; i < 3; i++)
      if (_isset(m_pwmEnablePin[i])) digitalWrite(m_pwmEnablePin[i], enable_active_high);
    // set zero to PWM
    setPwm(0, 0, 0);
  }

  void setPwm(float Ua, float Ub, float Uc) override {
    Ua = _constrain(Ua, 0.0f, voltage_limit);
    Ub = _constrain(Ub, 0.0f, voltage_limit);
    Uc = _constrain(Uc, 0.0f, voltage_limit);
    dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f);
    dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f);
    dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f);

    m_pwm[0]->write(dc_a);
    m_pwm[1]->write(dc_b);
    m_pwm[2]->write(dc_c);
  }

  virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) override {
    // disable if needed
    if (_isset(m_pwmEnablePin[0]) &&  _isset(m_pwmEnablePin[1])  && _isset(m_pwmEnablePin[2])) {
        digitalWrite(m_pwmEnablePin[0], sa == PhaseState::PHASE_ON ? enable_active_high : !enable_active_high);
        digitalWrite(m_pwmEnablePin[1], sb == PhaseState::PHASE_ON ? enable_active_high : !enable_active_high);
        digitalWrite(m_pwmEnablePin[2], sc == PhaseState::PHASE_ON ? enable_active_high : !enable_active_high);
    }
  }

  private:
    mbed::PwmOut* m_pwm[3];
    int m_pwmPin[3];
    int m_pwmEnablePin[3];
    bool enable_active_high;
};
