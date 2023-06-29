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
           uint16_t pwm_open)
    : motor_(motor),
      servo_(servo),
      pwm_close_(pmw_close),
      pwm_open_(pwm_open) {}

// 设置阀门状态
void Pump::setValve(ValveState_e state) {
  if (state == ValveState_e::CLOSE) {
    if (servo_->getMode() != ServoZX361D::NORMAL) {
      servo_->setMode(ServoZX361D::NORMAL);
    } else {
      if (servo_->setPwmTime(pwm_close_, 10)) {
        valve_state_ = state;
      }
    }
  } else if (state == ValveState_e::OPEN) {
    if (servo_->getMode() != ServoZX361D::NORMAL) {
      servo_->setMode(ServoZX361D::NORMAL);
    } else {
      if (servo_->setPwmTime(pwm_open_, 10)) {
        valve_state_ = state;
      }
    }
  }
}

// 气泵工作
void Pump::work(void) {
  setValve(Pump::ValveState_e::CLOSE);
  motor_->setSpeed(speed_);
}

// 气泵关闭
void Pump::off(void) {
  setValve(Pump::ValveState_e::OPEN);
  motor_->setSpeed(0);
}
