/**
 ******************************************************************************
 * @file    pump.cpp/h
 * @brief   Pump control program. 气泵控制程序
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/pump.h"

Pump::Pump(Motor* motor, ServoZX361D* servo, uint16_t pmw_close,
           float pwm_open_1, float pwm_open_2)
    : motor_(motor),
      servo_(servo),
      pwm_close_(pmw_close),
      pwm_open_1_(pwm_open_1),
      pwm_open_2_(pwm_open_2) {}

// 设置阀门状态
void Pump::set(const float& speed, ValveState_e state) {
  // 电机状态控制
  motor_speed_ = speed;
  motor_->setSpeed(speed);

  // 舵机控制
  if (state == ValveState_e::CLOSE) {
    if (servo_->getMode() != ServoZX361D::NORMAL) {
      servo_->setMode(ServoZX361D::NORMAL);
    } else {
      if (servo_->setPwmTime(pwm_close_, 10)) {
        valve_state_ = state;
      }
    }
  } else if (state == ValveState_e::OPEN_1) {
    if (servo_->getMode() != ServoZX361D::NORMAL) {
      servo_->setMode(ServoZX361D::NORMAL);
    } else {
      if (servo_->setPwmTime(pwm_open_1_, 10)) {
        valve_state_ = state;
      }
    }
  } else if (state == ValveState_e::OPEN_2) {
    if (servo_->getMode() != ServoZX361D::NORMAL) {
      servo_->setMode(ServoZX361D::NORMAL);
    } else {
      if (servo_->setPwmTime(pwm_open_2_, 10)) {
        valve_state_ = state;
      }
    }
  }
}
