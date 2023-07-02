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

J0Gimbal::J0Gimbal(Motor* jm0, Motor* gm_yaw, Motor* gm_pitch, IMU* j0_imu)
    : jm0_(jm0), gm_yaw_(gm_yaw), gm_pitch_(gm_pitch), j0_imu_(j0_imu) {}

// J0初始化
void J0Gimbal::initJ0(void) {
  init_.j0_finish = false;
  // 设置J0反馈角度为编码器单圈角度
  jm0_->motor_data_.angle = math::degNormalize180(
      (jm0_->motor_data_.ecd_angle - param_.j0_zero) / jm0_->ratio_);
  jm0_->resetFeedbackAngle(jm0_->motor_data_.angle);
  // 设置反馈数据源为陀螺仪
  jm0_->setFdbSrc(&j0_imu_->yaw(), &j0_imu_->wzWorld());
}

// yaw, pitch初始化
void J0Gimbal::initGM(void) {
  init_.yaw_finish = false;
  init_.pitch_finish = false;
  // 设置反馈数据源
  gm_yaw_->setFdbSrc(&gm_yaw_->kfAngle(), &gm_yaw_->kfSpeed());
  gm_pitch_->setFdbSrc(&gm_pitch_->kfAngle(), &gm_pitch_->kfSpeed());
}

// 全部初始化
void J0Gimbal::initAll(void) {
  initJ0();
  initGM();
}

// 设置云台角度(deg)
void J0Gimbal::setAngle(const float& j0, const float& yaw, const float& pitch) {
  ref_.j0 = j0;
  ref_.yaw = math::limit(yaw, param_.yaw_min, param_.yaw_max);
  ref_.pitch = math::limit(pitch, param_.pitch_min, param_.pitch_max);
}

// 设置云台角度增量(deg)
void J0Gimbal::addAngle(const float& j0, const float& yaw, const float& pitch) {
  // 设置目标状态
  setAngle(ref_.j0 + j0, ref_.yaw + yaw, ref_.pitch + pitch);
}

// 设置电机目标状态，更新反馈数据
void J0Gimbal::handle(void) {
  // J0初始化，更新反馈角度并回正
  if (!init_.j0_finish) {
    ref_.j0 = 0;
    init_.j0_finish = jm0_->connect_.check() &&
                      (fabs(gm_yaw_->motor_data_.angle) < init_.j0_thres);
  }
  // yaw初始化
  if (!init_.yaw_finish) {
    ref_.yaw -= 9e-2f;
    if (gm_yaw_->connect_.check() &&
        gm_yaw_->targetAngle() - gm_yaw_->realAngle() < -10) {
      // 初始化完成
      gm_yaw_->resetFeedbackAngle(init_.yaw_angle);
      ref_.yaw = 0;
      init_.yaw_finish = true;
    }
  }
  // pitch初始化
  if (!init_.pitch_finish) {
    ref_.pitch -= 9e-2f;
    if (gm_pitch_->connect_.check() &&
        gm_pitch_->targetAngle() - gm_pitch_->realAngle() < -10) {
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
  if (!gm_yaw_->connect_.check() && !gm_pitch_->connect_.check()) {
    initGM();
  }

  // 读取电机反馈
  fdb_.j0 = jm0_->realAngle();
  fdb_.j0_speed = jm0_->realSpeed();
  fdb_.yaw = gm_yaw_->realAngle();
  fdb_.yaw_speed = gm_yaw_->realSpeed();
  fdb_.pitch = gm_pitch_->realAngle();
  fdb_.pitch_speed = gm_pitch_->realSpeed();

  // 设置电机控制目标角度
  jm0_->setAngle(ref_.j0);
  gm_yaw_->setAngle(ref_.yaw);
  gm_pitch_->setAngle(ref_.pitch);
}
