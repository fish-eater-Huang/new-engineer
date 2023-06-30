/**
 ******************************************************************************
 * @file    pid.cpp/h
 * @brief   PID algorithm. PID算法实现
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "pid.h"
#include "math.h"

PID::PID(float kp, float ki, float kd, float i_max, float out_max,
         float d_filter_k)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      i_max_(i_max),
      out_max_(out_max),
      d_filter_(d_filter_k) {
  err_ = 0;
  err_sum_ = 0;
  last_err_ = 0;
}

void PID::reset(void) {
  err_ = 0;
  err_sum_ = 0;
  last_err_ = 0;
  pout_ = 0;
  iout_ = 0;
  dout_ = 0;
  d_filter_.reset();
}

float PID::calc(float ref, float fdb) {
  ref_ = ref;
  fdb_ = fdb;

  last_err_ = err_;
  err_ = ref_ - fdb_;
  err_sum_ = math::limit(err_sum_ + err_, -i_max_, i_max_);

  pout_ = kp_ * err_;
  iout_ = ki_ * err_sum_;
  dout_ = d_filter_.update(kd_ * (err_ - last_err_));
  output_ = math::limit(pout_ + iout_ + dout_, -out_max_, out_max_);

  return output_;
}
