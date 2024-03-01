#include "config.h"

#include <RPC.h>
#include <SPI.h>

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/MT6701/MagneticSensorMT6701SSI.h>

BLDCDriver3PWM drivers[] = {
  BLDCDriver3PWM(PIN_U_M1, PIN_V_M1, PIN_W_M1, PIN_EN_M1),
  BLDCDriver3PWM(PIN_U_M2, PIN_V_M2, PIN_W_M2, PIN_EN_M2),
  BLDCDriver3PWM(PIN_U_M3, PIN_V_M3, PIN_W_M3, PIN_EN_M3),
  BLDCDriver3PWM(PIN_U_M4, PIN_V_M4, PIN_W_M4, PIN_EN_M4),
  BLDCDriver3PWM(PIN_U_M5, PIN_V_M5, PIN_W_M5, PIN_EN_M5)
};

BLDCMotor motors[] = {
  BLDCMotor(MOTOR_PP),
  BLDCMotor(MOTOR_PP),
  BLDCMotor(MOTOR_PP),
  BLDCMotor(MOTOR_PP),
  BLDCMotor(MOTOR_PP)
};

MagneticSensorMT6701SSI sensors[] = {
  MagneticSensorMT6701SSI(PIN_MAG_CS1),
  MagneticSensorMT6701SSI(PIN_MAG_CS2),
  MagneticSensorMT6701SSI(PIN_MAG_CS3),
  MagneticSensorMT6701SSI(PIN_MAG_CS4),
  MagneticSensorMT6701SSI(PIN_MAG_CS5),
};

Commander command = Commander(RPC);

void doMotor1(char* cmd) { command.motor(&motors[0], cmd); }
void doMotor2(char* cmd) { command.motor(&motors[1], cmd); }
void doMotor3(char* cmd) { command.motor(&motors[2], cmd); }
void doMotor4(char* cmd) { command.motor(&motors[3], cmd); }
void doMotor5(char* cmd) { command.motor(&motors[4], cmd); }

void setup() {
  RPC.begin();
  RPC.println("starting M4...");

  SPI.begin();

  SimpleFOCDebug::enable(&RPC);

  pinMode(PIN_LED, OUTPUT);

  for (int i = 0; i < MOTORS; i++) {
    RPC.print("initialize motor ");
    RPC.println(i + 1);

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
  for (int i = 0; i < MOTORS; i++) {
    motors[i].loopFOC();
    motors[i].move();
  }
  command.run();
  digitalWrite(PIN_LED, (micros() & 0x100000) ? HIGH : LOW);
}
