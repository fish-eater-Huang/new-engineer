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

  // 设置阀门状态
  void setValve(ValveState_e state);
  // 电机转速
  float& speed(void) { return speed_; }

  // 气泵工作
  void work(void);
  // 气泵关闭
  void off(uint8_t i);

 private:
  Motor* motor_;
  ServoZX361D* servo_;

  ValveState_e valve_state_;

  float speed_;          // 电机转速
  uint16_t pwm_close_;   // 阀门关闭舵机PWM
  uint16_t pwm_open_1_;  // 阀门开启舵机PWM
  uint16_t pwm_open_2_;  // 阀门开启舵机PWM
};

#endif  // PUMP_H
