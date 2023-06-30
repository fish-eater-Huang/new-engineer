/**
 ******************************************************************************
 * @file    pid.cpp/h
 * @brief   PID algorithm. PID算法实现
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef PID_H
#define PID_H

#include "base/common/filter.h"

// PID类
class PID {
 public:
  PID(void) : PID(0, 0, 0, 0, 0) {}
  PID(float kp, float ki, float kd, float i_max, float out_max,
      float d_filter_k = 1);

  void reset(void);
  float calc(float ref, float fdb);

 public:
  float kp_, ki_, kd_;
  float i_max_, out_max_;
  float output_;

  // 微分项低通滤波器
  LowPassFilter d_filter_;

 protected:
  float ref_, fdb_;
  float err_, err_sum_, last_err_;
  float pout_, iout_, dout_;
};

#endif  // PID_H
