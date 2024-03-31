/*
 * Port of Arduino Code to Pico By Jon Durrant
  xArmServoController.h - Library for controlling xArm servos.
  Version 1.0.0
  Created by Chris Courson, July 9, 2020.
  Released into the public domain.
*/
#pragma once
#ifndef XARMSERVOCONTROLLER_H
#define XARMSERVOCONTROLLER_H

#include <stdarg.h>
#include <pico/stdlib.h>
#include "hardware/uart.h"

#define TIMEOUT 1000
#define byte uint8_t
#define delay sleep_ms
#define DEBUG_LINE 30

#define SIGNATURE               0x55
#define CMD_BEEP                0x00
#define CMD_SERVO_MOVE          0x03
#define CMD_ACTION_GROUP_RUN    0x06
#define CMD_ACTION_GROUP_STOP   0x07
#define CMD_ACTION_GROUP_END    0x08
#define CMD_ACTION_GROUP_SPEED  0x0B
#define CMD_GET_BATTERY_VOLTAGE 0x0f
#define CMD_SERVO_STOP          0x14
#define CMD_GET_SERVO_POSITION  0x15

enum xArmMode : int {xArm, LeArm};

struct xArmServo {
    int servo_id;
    unsigned position;
};

class xArmServoController {
  public:  
    xArmServoController(xArmMode mode, uart_inst_t * uart);

    void setPosition(int servo_id, unsigned position, unsigned duration = 1000, bool wait = false);
    void setPosition(xArmServo servo, unsigned duration = 1000, bool wait = false);
    void setPosition(xArmServo servos[], int count, unsigned duration = 1000, bool wait = false);

    int getPosition(int servo_id);
    int getPosition(xArmServo &servo);
    bool getPosition(xArmServo servos[], int count);
    
    void servoOff(int servo_id);
    void servoOff(int num, int servo_id, ...);
    void servoOff(xArmServo servo);
    void servoOff(xArmServo servos[], int count);
    void servoOff();

    void actionRun(int group, unsigned times = 1);
    void actionStop();
    void actionSpeed(int group, unsigned percent);
    bool actionIsRunning();
    bool serialEvent();

    int getBatteryVoltage();
    
    void beep();

  protected:
    uart_inst_t * pUart = NULL;
  
  private:
    byte _buffer[32];
    xArmMode xMode;

    bool actionRunning;
    
    unsigned clamp(unsigned v, unsigned lo, unsigned hi);
    unsigned clampServoLimits(int servo, unsigned value);
    void send(int cmd, int len = 0);
    int recv(int cmd);

    uint8_t lowByte(uint16_t x);
    uint8_t highByte(uint16_t x);

    void debugPrintBuffer(const char *title, const void * pBuffer, size_t bytes);
};

#endif
