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

Pump::Pump(Motor* motor, ServoPwm* pwm_servo, ServoZX361D* uart_servo,
           uint16_t pmw_close, float pwm_open_0, float pwm_open_1)
    : motor_(motor),
      pwm_servo_(pwm_servo),
      uart_servo_(uart_servo),
      pwm_close_(pmw_close),
      pwm_open_0_(pwm_open_0),
      pwm_open_1_(pwm_open_1) {}

// 初始化
void Pump::init(void) {
  motor_->setFdbSrc(&motor_->kfAngle(), &motor_->kfSpeed());
}

// 设置气泵电机转速
void Pump::setMotorSpeed(const float& speed) {
  motor_speed_ = speed;
  motor_->setSpeed(speed);
}

// 设置阀门状态
void Pump::setValve(ValveState_e state) {
  // 舵机控制
  if (pwm_servo_ != nullptr) {
    // PWM舵机
    if (state == ValveState_e::CLOSE) {
      pwm_servo_->setPwm(pwm_close_);
    } else if (state == ValveState_e::OPEN_0) {
      pwm_servo_->setPwm(pwm_open_0_);
    } else if (state == ValveState_e::OPEN_1) {
      pwm_servo_->setPwm(pwm_open_1_);
    }
    valve_state_ = state;
  } else if (uart_servo_ != nullptr) {
    // 总线舵机
    if (state == ValveState_e::CLOSE) {
      if (uart_servo_->getMode() != ServoZX361D::NORMAL) {
        uart_servo_->setMode(ServoZX361D::NORMAL);
      } else {
        if (uart_servo_->setPwmTime(pwm_close_, 10)) {
          valve_state_ = state;
        }
      }
    } else if (state == ValveState_e::OPEN_0) {
      if (uart_servo_->getMode() != ServoZX361D::NORMAL) {
        uart_servo_->setMode(ServoZX361D::NORMAL);
      } else {
        if (uart_servo_->setPwmTime(pwm_open_0_, 10)) {
          valve_state_ = state;
        }
      }
    } else if (state == ValveState_e::OPEN_1) {
      if (uart_servo_->getMode() != ServoZX361D::NORMAL) {
        uart_servo_->setMode(ServoZX361D::NORMAL);
      } else {
        if (uart_servo_->setPwmTime(pwm_open_1_, 10)) {
          valve_state_ = state;
        }
      }
    }
  }
}
