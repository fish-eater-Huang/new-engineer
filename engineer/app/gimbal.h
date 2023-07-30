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

#include "app/encoder.h"
#include "base/imu/imu.h"
#include "base/motor/motor.h"

// 机械臂云台类
class ArmGimbal {
 public:
  ArmGimbal(Motor* jm0, Motor* gm_pitch, IMU* j0_imu,
            AS5048Encoder* j0_encoder);

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

  // 获取J0编码器角度
  float j0EncoderAngle(void);

  // 设置电机目标状态，更新反馈数据
  void handle(void);

 public:
  // 云台/J0参数
  struct Param_t {
    // 外置编码器零点
    float ext_j0_zero = 42.0;
    // 电机编码器零点
    float jm0_zero = 8.8;

    // 云台角度限位(°)
    float pitch_min = -45;
    float pitch_max = 5;
  } param_;

  // 初始化相关
  struct Init_t {
    bool j0_finish;
    bool pitch_finish;

    float j0_thres = 5;
    float pitch_angle = 25;

    float jm0_encoder_offset;
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
  // J0编码器指针
  AS5048Encoder* j0_encoder_;
};

#endif  // GIMBAL_H