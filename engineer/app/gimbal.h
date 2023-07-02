/**
 ******************************************************************************
 * @file    gimbal.cpp/h
 * @brief   Gimbal control. 云台控制
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef GIMBAL_H
#define GIMBAL_H

#include "app/arm.h"
#include "base/imu/imu.h"
#include "base/motor/motor.h"

// J0云台类
class J0Gimbal {
 public:
  J0Gimbal(Motor* jm0, Motor* gm_yaw, Motor* gm_pitch, IMU* j0_imu);

  // J0初始化
  void initJ0(void);
  // yaw, pitch初始化
  void initGM(void);
  // 全部初始化
  void initAll(void);

  // 设置云台角度(deg)
  void setAngle(const float& j0, const float& yaw, const float& pitch);
  // 设置云台角度增量(deg)
  void addAngle(const float& j0, const float& yaw, const float& pitch);

  // 设置电机目标状态，更新反馈数据
  void handle(void);

 public:
  // 云台/J0参数
  struct Param_t {
    // 编码器零点
    const float j0_zero = 0;

    // 云台角度限位(°)
    const float yaw_min = -180;
    const float yaw_max = 180;
    const float pitch_min = -90;
    const float pitch_max = 90;

    // 初始化
    const float j0_speed_limit = 360;
    const float gm_speed_limit = 720;
  } param_;

  // 初始化相关
  struct Init_t {
    bool j0_finish;
    bool yaw_finish;
    bool pitch_finish;

    const float j0_thres = 5;
    const float yaw_angle = -90;
    const float pitch_angle = -90;
  } init_;

  // 目标状态数据
  struct Ref_t {
    float j0;
    float yaw;
    float pitch;
  } ref_;

  // 反馈状态数据
  struct Fdb_t {
    float j0;
    float yaw;
    float pitch;
    float j0_speed;
    float yaw_speed;
    float pitch_speed;
  } fdb_;

 private:
  // 电机指针
  Motor *jm0_, *gm_yaw_, *gm_pitch_;
  // imu指针
  IMU* j0_imu_;
};

// 机械臂云台类
class ArmGimbal {
 public:
  ArmGimbal(Motor* jm0, Motor* gm_pitch, IMU* j0_imu, Arm* arm);

  // J0初始化
  void initJ0(void);
  // pitch初始化
  void initGM(void);
  // 全部初始化
  void initAll(void);

  // 设置云台角度(deg)
  void setAngle(const float& j0, const float& pitch);
  // 设置云台角度增量(deg)
  void addAngle(const float& j0, const float& pitch);

  // 设置电机目标状态，更新反馈数据
  void handle(void);

 public:
  // 云台/J0参数
  struct Param_t {
    // 编码器零点
    const float j0_zero = 0;

    // 云台角度限位(°)
    const float pitch_min = -90;
    const float pitch_max = 90;

    // 初始化
    const float j0_speed_limit = 360;
    const float gm_speed_limit = 720;
  } param_;

  // 初始化相关
  struct Init_t {
    bool j0_finish;
    bool pitch_finish;

    const float j0_thres = 5;
    const float pitch_angle = -90;
  } init_;

  // 目标状态数据
  struct Ref_t {
    float j0;
    float pitch;
  } ref_;

  // 反馈状态数据
  struct Fdb_t {
    float j0;
    float pitch;
    float j0_speed;
    float pitch_speed;
  } fdb_;

 private:
  // 电机指针
  Motor *jm0_, *gm_pitch_;
  // imu指针
  IMU* j0_imu_;
  // 机械臂指针
  Arm* arm_;
};

#endif  // GIMBAL_H