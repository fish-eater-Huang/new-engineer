/**
 ******************************************************************************
 * @file    mahony.cpp/h
 * @brief   Mahony algorithm. Mahony算法实现
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "base/imu/mahony.h"

#include "base/common/matrix.h"

// Mahony algorithm, sensor data fusion, update quaternion
// Mahony算法，传感器数据融合，四元数更新
void Mahony::update(float q[4], float ws[3], float as[3], float ms[3]) {
  // Update sensor data
  q_ = Matrixf<4, 1>(q);
  ws_ = Matrixf<3, 1>(ws);
  as_ = Matrixf<3, 1>(as);
  ms_ = Matrixf<3, 1>(ms);

  // Calculate rotate matrix
  R_[0][0] = 1 - 2.f * (q[2] * q[2] + q[3] * q[3]);
  R_[0][1] = 2.f * (q[1] * q[2] - q[0] * q[3]);
  R_[0][2] = 2.f * (q[1] * q[3] + q[0] * q[2]);
  R_[1][0] = 2.f * (q[1] * q[2] + q[0] * q[3]);
  R_[1][1] = 1 - 2.f * (q[1] * q[1] + q[3] * q[3]);
  R_[1][2] = 2.f * (q[2] * q[3] - q[0] * q[1]);
  R_[2][0] = 2.f * (q[1] * q[3] - q[0] * q[2]);
  R_[2][1] = 2.f * (q[2] * q[3] + q[0] * q[1]);
  R_[2][2] = 1 - 2.f * (q[1] * q[1] + q[2] * q[2]);

  // Calculate acceleration(g) & magnetic field reference
  _gs_ = R_.trans() * _gw_;  // -gs=R^T*(-gw)
  mr_[0][0] = sqrt(ms[0] * ms[0] + ms[1] * ms[1]);
  mr_[1][0] = 0;
  mr_[2][0] = ms[2];

  // calculate error between measure & reference of acceleration & magnetic
  eg_ = vector3f::cross(as_, _gs_);
  em_ = vector3f::cross(ms_, mr_);

  // wu(update)=ws+kg*eg+km*em
  if (fabs(as_.norm() - _gw_.norm()) < g_thres_) {
    wu_ = ws_ + kg_ * eg_ + km_ * em_;
  } else {
    wu_ = ws_ + km_ * em_;
  }

  // calculate status in world coordinate
  ww_ = R_ * ws_;
  aw_ = R_ * as_ - _gw_;
  mw_ = R_ * ms_;

  // update quaternion
  dq_[0][0] =
      0.5f * (-q[1] * wu_[0][0] - q[2] * wu_[1][0] - q[3] * wu_[2][0]) * dt_;
  dq_[1][0] =
      0.5f * (q[0] * wu_[0][0] - q[3] * wu_[1][0] + q[2] * wu_[2][0]) * dt_;
  dq_[2][0] =
      0.5f * (q[3] * wu_[0][0] + q[0] * wu_[1][0] - q[1] * wu_[2][0]) * dt_;
  dq_[3][0] =
      0.5f * (-q[2] * wu_[0][0] + q[1] * wu_[1][0] + q[0] * wu_[2][0]) * dt_;
  q_ += dq_;
  q_ /= q_.norm();
  memcpy(q, q_[0], 4 * sizeof(float));
}

// Set dt
// 设置时间步长
void Mahony::setDt(float dt) {
  dt_ = dt;
}

// Set sensor fusion coeficient
// 设置传感器融合系数
void Mahony::setK(float kg, float km) {
  kg_ = kg;
  km_ = km;
}
