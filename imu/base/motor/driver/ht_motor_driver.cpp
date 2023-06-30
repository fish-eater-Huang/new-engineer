/**
 ******************************************************************************
 * @file    ht_motor_driver.cpp/h
 * @brief   HT motor driver. 海泰电机驱动
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "base/motor/driver/ht_motor_driver.h"
#include "base/common/math.h"
#include "lib/arm_math/arm_math.h"

namespace ht04 {

// Converts a float to an unsigned int, given range and number of bits
uint16_t float2uint(float x, float x_min, float x_max, uint8_t bits);
// converts unsigned int to float, given range and number of bits
float uint2float(int x_int, float x_min, float x_max, int bits);

};  // namespace ht04

// Converts a float to an unsigned int, given range and number of bits
uint16_t ht04::float2uint(float x, float x_min, float x_max, uint8_t bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// converts unsigned int to float, given range and number of bits
float ht04::uint2float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// transmit motor mode command
bool HT04Driver::setCmd(ht04::CmdType_e cmd) {
  if (cmd_cnt_ > 5 || (cmd_cnt_ > 0 && cmd_list_[cmd_cnt_ - 1] == cmd)) {
    return false;
  }
  cmd_list_[cmd_cnt_] = cmd;
  cmd_cnt_++;
  return true;
}

// transmit motor control parameter
// p: target position(rad)
// v: target velocity(rad/s)
// kp/kv: control gain
// t: feedforward torque
// T = kp*(p-p_fdb)+kv*(v-v_fdb)+t
void HT04Driver::setControlParam(float p, float v, float kp, float kv,
                                 float t_ff) {
  // control parameter
  p = math::limit(p, ht04::p_min, ht04::p_max);
  v = math::limit(v, ht04::v_min, ht04::v_max);
  kp = math::limit(kp, ht04::kp_min, ht04::kp_max);
  kv = math::limit(kv, ht04::kv_min, ht04::kv_max);
  t_ff = math::limit(t_ff, ht04::t_ff_min, ht04::t_ff_max);
  tx_param_.p = ht04::float2uint(p, ht04::p_min, ht04::p_max, 16);
  tx_param_.v = ht04::float2uint(v, ht04::v_min, ht04::v_max, 12);
  tx_param_.kp = ht04::float2uint(kp, ht04::kp_min, ht04::kp_max, 12);
  tx_param_.kv = ht04::float2uint(kv, ht04::kv_min, ht04::kv_max, 12);
  tx_param_.t_ff = ht04::float2uint(t_ff, ht04::t_ff_min, ht04::t_ff_max, 12);
}

// set torque, torq(N·m)
void HT04Driver::setTorque(const float& torque) {
  setControlParam(0, 0, 0, 0, torque);
}

// can transmit message
void HT04Driver::canTxMsg(void) {
  // can message header
  can_tx_header_.IDE = CAN_ID_STD;
  can_tx_header_.RTR = CAN_RTR_DATA;
  can_tx_header_.DLC = 8;
  can_tx_header_.StdId = id_;

  if (cmd_cnt_ > 0) {
    setTorque(0);
    memset(can_tx_data_, 0xff, 8);
    can_tx_data_[7] = 0xfb + cmd_list_[cmd_cnt_ - 1];
    if (HAL_GetTick() - cmd_tick_ > cmd_interval_) {
      HAL_CAN_AddTxMessage(hcan_, &can_tx_header_, can_tx_data_,
                           &can_tx_mail_box_);
      memmove(cmd_list_, (ht04::CmdType_e*)cmd_list_ + 1,
              4 * sizeof(ht04::CmdType_e));
      cmd_list_[cmd_cnt_ - 1] = ht04::NO_CMD;
      cmd_cnt_--;
      cmd_tick_ = HAL_GetTick();
    } else {
      can_tx_data_[0] = tx_param_.p >> 8;
      can_tx_data_[1] = tx_param_.p & 0xFF;
      can_tx_data_[2] = tx_param_.v >> 4;
      can_tx_data_[3] = ((tx_param_.v & 0xF) << 4) | (tx_param_.kp >> 8);
      can_tx_data_[4] = tx_param_.kp & 0xFF;
      can_tx_data_[5] = tx_param_.kv >> 4;
      can_tx_data_[6] = ((tx_param_.kv & 0xF) << 4) | (tx_param_.t_ff >> 8);
      can_tx_data_[7] = tx_param_.t_ff & 0xff;
      HAL_CAN_AddTxMessage(hcan_, &can_tx_header_, can_tx_data_,
                           &can_tx_mail_box_);
    }
  } else {
    setTorque(motor_->intensity_float_);
    can_tx_data_[0] = tx_param_.p >> 8;
    can_tx_data_[1] = tx_param_.p & 0xFF;
    can_tx_data_[2] = tx_param_.v >> 4;
    can_tx_data_[3] = ((tx_param_.v & 0xF) << 4) | (tx_param_.kp >> 8);
    can_tx_data_[4] = tx_param_.kp & 0xFF;
    can_tx_data_[5] = tx_param_.kv >> 4;
    can_tx_data_[6] = ((tx_param_.kv & 0xF) << 4) | (tx_param_.t_ff >> 8);
    can_tx_data_[7] = tx_param_.t_ff & 0xff;
    HAL_CAN_AddTxMessage(hcan_, &can_tx_header_, can_tx_data_,
                         &can_tx_mail_box_);
  }
}

// Check CAN channel and CAN message header id
// 校验CAN通道和ID
bool HT04Driver::canRxMsgCheck(CAN_HandleTypeDef* hcan,
                               CAN_RxHeaderTypeDef rx_header) {
  // return hcan == hcan_ && rx_header.StdId == id_;
  return hcan == hcan_ && rx_header.StdId == 0;
}

// Receive feedback data message callback. Called in
// HAL_CAN_RxFifo0MsgPendingCallback()
// 电调反馈信息接受回调，在HAL_CAN_RxFifo0MsgPendingCallback中调用
void HT04Driver::canRxMsgCallback(CAN_HandleTypeDef* hcan,
                                  CAN_RxHeaderTypeDef rx_header,
                                  uint8_t rx_data[8]) {
  if (motor_->info_.type != Motor::HT04 || rx_data[0] != id_) {
    return;
  }

  // unpack feedback data
  // 反馈数据解包
  uint16_t uint_p, uint_v, uint_t;
  uint_p = (rx_data[1] << 8) | (rx_data[2]);
  uint_v = (rx_data[3] << 4) | (rx_data[4] >> 4);
  uint_t = ((rx_data[4] & 0x0F) << 8) | (rx_data[5]);
  raw_data_.position = ht04::uint2float(uint_p, ht04::p_min, ht04::p_max, 16);
  raw_data_.velocity = ht04::uint2float(uint_v, ht04::v_min, ht04::v_max, 12);
  raw_data_.torq = ht04::uint2float(uint_t, ht04::t_min, ht04::t_max, 12);

  // Encoder angle(deg)
  // 编码器角度
  motor_->motor_data_.ecd_angle = math::rad2deg(raw_data_.position);
  // Use incremental calculation to deals with encoder overflow and underflow
  // 增量计算处理编码器上下溢问题
  float delta =
      motor_->motor_data_.ecd_angle - motor_->motor_data_.last_ecd_angle;
  delta = math::degNormalize180(delta) / motor_->ratio_;
  motor_->motor_data_.angle += delta;  // deg
  // feedback rotational speed
  // 反馈转速
  motor_->motor_data_.rotate_speed =
      math::radps2dps(raw_data_.velocity) / motor_->ratio_;  // dps
  // Update current
  // 更新转矩电流
  motor_->motor_data_.current = raw_data_.torq;
  // update encoder angle record
  // 更新编码器角度记录
  motor_->motor_data_.last_ecd_angle = motor_->motor_data_.ecd_angle;

  motor_->rxCallback();
}
