#include "config.h"

#include <RPC.h>
#include <SPI.h>

#include <SimpleFOC.h>

#include <vector>

std::vector<int> usedMotors = {0, 2, 3, 4};

#ifdef DRIVER_MBED
#include "mbed_driver.h"
MbedDriver3PWM drivers[] = {
  MbedDriver3PWM(PIN_U_M1, PIN_V_M1, PIN_W_M1, PIN_EN_M1),
  MbedDriver3PWM(PIN_U_M2, PIN_V_M2, PIN_W_M2, PIN_EN_M2),
  MbedDriver3PWM(PIN_U_M3, PIN_V_M3, PIN_W_M3, PIN_EN_M3),
  MbedDriver3PWM(PIN_U_M4, PIN_V_M4, PIN_W_M4, PIN_EN_M4),
  MbedDriver3PWM(PIN_U_M5, PIN_V_M5, PIN_W_M5, PIN_EN_M5)
};
#endif

#ifdef DRIVER_STM32
#include "stm32_driver.h"
GigaBLDCDriver3PWM drivers[] = {
  GigaBLDCDriver3PWM(PIN_U_M1, PIN_V_M1, PIN_W_M1, PIN_EN_M1),
  GigaBLDCDriver3PWM(PIN_U_M2, PIN_V_M2, PIN_W_M2, PIN_EN_M2),
  GigaBLDCDriver3PWM(PIN_U_M3, PIN_V_M3, PIN_W_M3, PIN_EN_M3),
  GigaBLDCDriver3PWM(PIN_U_M4, PIN_V_M4, PIN_W_M4, PIN_EN_M4),
  GigaBLDCDriver3PWM(PIN_U_M5, PIN_V_M5, PIN_W_M5, PIN_EN_M5)
};
#endif

#ifdef DRIVER_PORTENTA
#include "portenta_driver.h"
PortentaDriver3PWM drivers[] = {
  PortentaDriver3PWM(PIN_U_M1, PIN_V_M1, PIN_W_M1, PIN_EN_M1),
  PortentaDriver3PWM(PIN_U_M2, PIN_V_M2, PIN_W_M2, PIN_EN_M2),
  PortentaDriver3PWM(PIN_U_M3, PIN_V_M3, PIN_W_M3, PIN_EN_M3),
  PortentaDriver3PWM(PIN_U_M4, PIN_V_M4, PIN_W_M4, PIN_EN_M4),
  PortentaDriver3PWM(PIN_U_M5, PIN_V_M5, PIN_W_M5, PIN_EN_M5)
};
#endif

BLDCMotor motors[] = {
  BLDCMotor(MOTOR_PP),
  BLDCMotor(MOTOR_PP),
  BLDCMotor(MOTOR_PP),
  BLDCMotor(MOTOR_PP),
  BLDCMotor(MOTOR_PP)
};

#ifdef SENSOR_MT6701
#include "mt6701.h"
SensorMT6701SSI sensors[] = {
  SensorMT6701SSI(PIN_MAG_CS1),
  SensorMT6701SSI(PIN_MAG_CS2),
  SensorMT6701SSI(PIN_MAG_CS3),
  SensorMT6701SSI(PIN_MAG_CS4),
  SensorMT6701SSI(PIN_MAG_CS5),
};
#endif

#ifdef SENSOR_MT6835
#include "mt6835.h"
SensorMT6835 sensors[] = {
  SensorMT6835(PIN_MAG_CS1),
  SensorMT6835(PIN_MAG_CS2),
  SensorMT6835(PIN_MAG_CS3),
  SensorMT6835(PIN_MAG_CS4),
  SensorMT6835(PIN_MAG_CS5),
};
#endif

Commander command = Commander();

void doMotor1(char* cmd) { command.motor(&motors[0], cmd); }
void doMotor2(char* cmd) { command.motor(&motors[1], cmd); }
void doMotor3(char* cmd) { command.motor(&motors[2], cmd); }
void doMotor4(char* cmd) { command.motor(&motors[3], cmd); }
void doMotor5(char* cmd) { command.motor(&motors[4], cmd); }

void setup() {
  Stream *stream = 0;
  if (HAL_GetCurrentCPUID() == CM7_CPUID) {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {}
    Serial.println("starting M7...");
    stream = &Serial;
  } else {
    RPC.begin();
    RPC.println("starting M4...");
    stream = &RPC;
  }

  SPI.begin();

  command.com_port = stream;
  SimpleFOCDebug::enable(stream);

  pinMode(PIN_LED, OUTPUT);

  for (const int& i : usedMotors) {
    stream->print("initialize motor ");
    stream->println(i + 1);

    sensors[i].init(&SPI);

    drivers[i].voltage_power_supply = VOLTAGE_POWER_SUPPLY;
    drivers[i].init();

    motors[i].voltage_sensor_align = VOLTAGE_SENSOR_ALIGN;
    motors[i].velocity_index_search = VELOCITY_INDEX_SEARCH;
    motors[i].controller = MotionControlType::angle;

    motors[i].linkSensor(&sensors[i]);
    motors[i].linkDriver(&drivers[i]);
    motors[i].init();
    motors[i].initFOC();

    switch (i) {
      case 0: command.add('A', doMotor1,  (char*)"Motor 1"); break;
      case 1: command.add('B', doMotor2,  (char*)"Motor 2"); break;
      case 2: command.add('C', doMotor3,  (char*)"Motor 3"); break;
      case 3: command.add('D', doMotor4,  (char*)"Motor 4"); break;
      case 4: command.add('E', doMotor5,  (char*)"Motor 5"); break;
      default: break;
    }
  }
}

void loop() {
  for (const int& i : usedMotors) {
    motors[i].loopFOC();
    motors[i].move();
  }
  command.run();
  digitalWrite(PIN_LED, (micros() & 0x100000) ? HIGH : LOW);
}
