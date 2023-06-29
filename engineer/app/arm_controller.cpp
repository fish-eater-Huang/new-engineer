/**
 ******************************************************************************
 * @file    arm_controller.cpp/h
 * @brief   6-DOF arm controller program. 6轴机械臂控制器程序
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/arm_controller.h"

// 机械臂控制器构造函数
ArmController::ArmController(IMU imu[3]) {
  imu_[0] = &imu[0];
  imu_[1] = &imu[1];
  imu_[2] = &imu[2];
}

// 设置机械臂控制器位置偏置值
void ArmController::setOffset(float dx, float dy, float dz) {
  offset_.x = dx;
  offset_.y = dy;
  offset_.z = dz;
}

// 机械臂控制器处理函数
void ArmController::handle(void) {
  // 目标姿态
  raw_.yaw = math::deg2rad(imu_[2]->yaw());
  raw_.pitch = math::deg2rad(imu_[2]->pitch());
  raw_.roll = math::deg2rad(imu_[2]->roll());
  ref_.yaw = raw_.yaw;  // todo
  ref_.pitch = raw_.pitch;
  ref_.roll = raw_.roll;

  // 目标位置
  Matrixf<3, 1> p11 = matrixf::zeros<3, 1>();
  p11[0][0] = param_.l[0];  // [l1;0;0]
  Matrixf<3, 1> p22 = matrixf::zeros<3, 1>();
  p22[0][0] = param_.l[1];  // [l2;0;0]
  float rpy1[3] = {math::deg2rad(imu_[0]->yaw()),
                   math::deg2rad(imu_[0]->pitch()),
                   math::deg2rad(imu_[0]->roll())};
  float rpy2[3] = {math::deg2rad(imu_[1]->yaw()),
                   math::deg2rad(imu_[1]->pitch()),
                   math::deg2rad(imu_[1]->roll())};
  Matrixf<3, 3> R01 = robotics::rpy2r(rpy1);
  Matrixf<3, 3> R02 = robotics::rpy2r(rpy2);
  Matrixf<3, 1> p = R01 * p11 + R02 * p22;
  raw_.x = p[0][0];
  raw_.y = p[1][0];
  raw_.z = p[2][0];

  // 目标位置+偏置
  ref_.x = raw_.x + offset_.x;
  ref_.y = raw_.y + offset_.y;
  ref_.z = raw_.z + offset_.z;

  // 目标状态T矩阵
  float ref_p[3] = {ref_.x, ref_.y, ref_.z};
  float ref_rpy[3] = {ref_.yaw, ref_.pitch, ref_.roll};
  ref_.T = robotics::rp2t(robotics::rpy2r(ref_rpy), ref_p);
}
