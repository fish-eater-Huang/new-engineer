/**
 ******************************************************************************
 * @file    arm_controller.cpp/h
 * @brief   6-DOF arm controller program. 6轴机械臂控制器程序
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include "app/board_comm.h"  // todo
#include "base/common/connect.h"
#include "base/imu/imu.h"
#include "base/robotics/robotics.h"

// 机械臂控制器(3*imu)
class ArmController {
 public:
  // 构造函数
  ArmController(IMU imu[3], BoardComm* comm);

  // 设置偏置值
  void setOffset(float dx, float dy, float dz);

  // 控制器处理函数
  void handle(void);

 public:
  Connect connect_;

  // 目标状态
  struct Ref_t {
    Matrixf<4, 4> T;

    float x, y, z;           // m
    float yaw, pitch, roll;  // rad
  } ref_;

  // 状态偏置
  struct Raw_t {
    float x, y, z;           // m
    float yaw, pitch, roll;  // rad
  } raw_;

  // 状态偏置
  struct Offset_t {
    float x, y, z;           // m
    float yaw, pitch, roll;  // rad
  } offset_;

 private:
  IMU* imu_[3];
  BoardComm* comm_;  // todo

  struct Param {
    float l[2] = {0.3f, 0.27f};
  } param_;
};

#endif  // ARM_CONTROLLER_H
