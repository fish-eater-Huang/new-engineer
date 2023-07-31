/**
 ******************************************************************************
 * @file    gimbal.cpp/h
 * @brief   Gimbal control. 云台控制
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/gimbal.h"
#include "base/common/math.h"
#include "lib/arm_math/arm_math.h"

ArmGimbal::ArmGimbal(Motor* jm0, Motor* gm_pitch, IMU* j0_imu,
                     AS5048Encoder* j0_encoder)
    : jm0_(jm0),
      gm_pitch_(gm_pitch),
      j0_imu_(j0_imu),
      j0_encoder_(j0_encoder) {}

// J0初始化
void ArmGimbal::initJ0(void) {
  init_.j0_finish = false;
  // 设置J0反馈角度为编码器单圈角度
  if (j0_encoder_->connect_.check()) {
    jm0_->motor_data_.angle = math::degNormalize180(
        (j0_encoder_->rx_data_.deg - param_.ext_j0_zero) / jm0_->ratio_);
    init_.j0_encoder_flag = true;
    init_.jm0_encoder_offset =
        (j0_encoder_->rx_data_.deg - param_.ext_j0_zero) -
        jm0_->motor_data_.ecd_angle;
  } else {
    jm0_->motor_data_.angle = math::degNormalize180(
        (jm0_->motor_data_.ecd_angle - param_.jm0_zero) / jm0_->ratio_);
    init_.j0_encoder_flag = false;
    init_.jm0_encoder_offset = 0;
  }
  jm0_->resetFeedbackAngle(jm0_->motor_data_.angle);
  // 设置反馈数据源为陀螺仪
  jm0_->setFdbSrc(&j0_imu_->yaw(), &j0_imu_->wzWorld());
}

// yaw, pitch初始化
void ArmGimbal::initGM(void) {
  init_.pitch_finish = false;
  gm_pitch_->setFdbSrc(&gm_pitch_->kfAngle(), &gm_pitch_->kfSpeed());
  fdb_.pitch = gm_pitch_->realAngle();
  ref_.pitch = fdb_.pitch;
}

// 全部初始化
void ArmGimbal::initAll(void) {
  initJ0();
  initGM();
}

// 设置云台角度(deg)
void ArmGimbal::setAngle(const float& j0, const float& pitch) {
  ref_.j0 = j0;
  ref_.pitch = math::limit(pitch, param_.pitch_min, param_.pitch_max);
}

// 设置云台角度增量(deg)
void ArmGimbal::addAngle(const float& j0, const float& pitch) {
  // 设置目标状态
  setAngle(ref_.j0 + j0, ref_.pitch + pitch);
}

// 获取J0编码器角度
float ArmGimbal::j0EncoderAngle(void) {
  if (init_.j0_encoder_flag) {
    return math::degNormalize180(
        (jm0_->motor_data_.ecd_angle + init_.jm0_encoder_offset) /
        jm0_->ratio_);
  }
  return math::degNormalize180((jm0_->motor_data_.ecd_angle - param_.jm0_zero) /
                               jm0_->ratio_);
}

// 设置电机目标状态，更新反馈数据
void ArmGimbal::handle(void) {
  // J0初始化，更新反馈角度并回正
  if (!init_.j0_finish) {
    ref_.j0 =
        fdb_.j0 - fmin(fabs(jm0_->motor_data_.angle), 90.0f / jm0_->ppid_.kp_) *
                      math::sign(jm0_->motor_data_.angle);
    init_.j0_finish = (jm0_->connect_.check() &&
                       fabs(jm0_->motor_data_.angle) < init_.j0_thres);
  }
  // pitch初始化
  if (!init_.pitch_finish) {
    if (gm_pitch_->mode_ != Motor::POWEROFF &&
        gm_pitch_->mode_ != Motor::STOP) {
      ref_.pitch += 6e-2f;
      ref_.pitch = math::limitMax(ref_.pitch, fdb_.pitch + 20);
      gm_pitch_->targetAngle() = ref_.pitch;
    }
    if (gm_pitch_->connect_.check() &&
        gm_pitch_->targetAngle() - gm_pitch_->realAngle() > 10 &&
        gm_pitch_->motor_data_.current * gm_pitch_->ratio_ > 3000) {
      // 初始化完成
      gm_pitch_->resetFeedbackAngle(init_.pitch_angle);
      ref_.pitch = 0;
      init_.pitch_finish = true;
    }
  }

  // J0电机离线处理
  if (!jm0_->connect_.check()) {
    initJ0();
  }
  // 云台电机离线处理
  if (!gm_pitch_->connect_.check()) {
    initGM();
  }

  // 读取电机反馈
  fdb_.j0 = jm0_->realAngle();
  fdb_.j0_speed = jm0_->realSpeed();
  fdb_.pitch = gm_pitch_->realAngle();
  fdb_.pitch_speed = gm_pitch_->realSpeed();

  // 设置电机控制目标角度
  jm0_->setAngle(ref_.j0);
  gm_pitch_->setAngle(ref_.pitch);
}
