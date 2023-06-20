/**
 ******************************************************************************
 * @file    control.cpp/h
 * @brief   Robot control design. 机器人控制设计（模式/键位）
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef CONTROL_H
#define CONTROL_H

#include "app/imu_monitor.h"
#include "base/common/matrix.h"

// 控制初始化
void controlInit(void);
// 控制主循环
void controlLoop(void);

// 机械臂控制器(3*imu)
class ArmController {
 public:
  // 构造函数
  ArmController(IMU imu[3]);

  // 设置偏置值
  void setOffset(float dx, float dy, float dz);

  // 控制器处理函数
  void handle(void);

 public:
  // 目标状态
  struct Ref_t {
    Matrixf<4, 4> T;

    float x, y, z;           // m
    float yaw, pitch, roll;  // rad
  } ref_;

  // 状态偏置
  struct Offset_t {
    float x, y, z;
  } offset_;

 private:
  IMU* imu_[3];

  struct Para {
    float l[2] = {0.3f, 0.24f};  // {0.3f, 0.24f};
  } para_;
};

#endif  // CONTROL_H