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

#ifndef MAHONY_H
#define MAHONY_H

#include "base/common/matrix.h"

const float gravity_accel = 9.8f;

class Mahony {
 public:
  // Initialize & config parameter(time step & fusion coefficient)
  // 初始化，配置参数（时间步长，数据融合系数）
  Mahony(float dt, float kg, float km, float g_thres)
      : dt_(dt), kg_(kg), km_(km), g_thres_(g_thres) {}

  // Mahony algorithm, sensor data fusion, update quaternion
  // Mahony算法，传感器数据融合，四元数更新
  void update(float q[4], float ws[3], float as[3], float ms[3]);

  // Set dt
  // 设置时间步长
  void setDt(float dt);

  // Set sensor fusion corfficient
  // 设置数据融合系数
  void setK(float kg, float km);

 public:
  Matrixf<4, 1> q_;             // quaternion
  Matrixf<3, 1> ws_, as_, ms_;  // gyro/accleration/magnet(sensor)
  Matrixf<3, 3> R_;             // rotate matrix
  Matrixf<3, 1> ww_, aw_, mw_;  // gyro & acceleration(no g) & magnet (world)

 private:
  Matrixf<4, 1> dq_;       // quaternion increment
  Matrixf<3, 1> _gs_;      // minus gravityaccleration (sensor)
  Matrixf<3, 1> mr_;       // magnet reference
  Matrixf<3, 1> eg_, em_;  // error vector, ea=cross(-am,g), em=cross(um,um~)
  Matrixf<3, 1> wu_;       // wu(update)=ws+kg*ea+km*em

  float dt_;       // update time step;
  float kg_, km_;  // data fusion coefficient

  // -g(world)
  // const float gravity_accel_ = 9.8f;
  Matrixf<3, 1> _gw_ = Matrixf<3, 1>((float[3]){0, 0, gravity_accel});
  // threshold to refuse acceleration fusion
  const float g_thres_;
};

#endif  // MAHONY_H