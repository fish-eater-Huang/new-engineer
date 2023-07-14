/**
 ******************************************************************************
 * @file    pump.cpp/h
 * @brief   Pump control program. 气泵控制程序
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef PUMP_H
#define PUMP_H

#include "base/motor/motor.h"
#include "base/servo/servo.h"

class Pump {
 public:
  typedef enum ValveState {
    CLOSE,
    OPEN_1,
    OPEN_2,
  } ValveState_e;

 public:
  Pump(Motor* motor, ServoZX361D* servo, uint16_t pmw_close, float pwm_open_1,
       float pwm_open_2);

  // 设置气泵状态
  void set(const float& speed, ValveState_e state);

 public:
  float motor_speed_;         // 电机转速
  ValveState_e valve_state_;  // 阀门状态

 private:
  Motor* motor_;
  ServoZX361D* servo_;

  uint16_t pwm_close_;   // 阀门关闭舵机PWM
  uint16_t pwm_open_1_;  // 阀门开启舵机PWM
  uint16_t pwm_open_2_;  // 阀门开启舵机PWM
};

#endif  // PUMP_H
