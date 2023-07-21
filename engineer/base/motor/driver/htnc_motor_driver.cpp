/**
 ******************************************************************************
 * @file    htnc_motor_driver.cpp/h
 * @brief   Driver program for HT-NC motor. 海泰NC协议电机驱动
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "base/motor/driver/htnc_motor_driver.h"
#include "base/common/math.h"

// master id: motor->board
// slave id: board->motor
HTNCMotorDriver::HTNCMotorDriver(Motor* motor, CAN_HandleTypeDef* hcan,
                                 uint32_t master_id, uint32_t slave_id)
    : motor_(motor), hcan_(hcan), master_id_(master_id), slave_id_(slave_id) {
  if (hcan_ == &hcan1) {
    motor_->CANIdConfig(1, slave_id_);
  } else if (hcan_ == &hcan2) {
    motor_->CANIdConfig(2, slave_id_);
  }
}

// Set motor mode command
bool HTNCMotorDriver::setCmd(HTNCMotorDriver::Cmd_e cmd) {
  if (cmd_.cnt > 5 || (cmd_.cnt > 0 && cmd_.list[cmd_.cnt - 1] == cmd)) {
    return false;
  }
  cmd_.list[cmd_.cnt] = cmd;
  cmd_.cnt++;
  return true;
}

// Transmit CAN message
void HTNCMotorDriver::canTxMsg(void) {
  // can message header
  can_tx_header_.IDE = CAN_ID_STD;
  can_tx_header_.RTR = CAN_RTR_DATA;
  can_tx_header_.DLC = 8;
  can_tx_header_.StdId = slave_id_;

  // transmit command pack
  if (cmd_.cnt > 0 && HAL_GetTick() - cmd_.tick > cmd_.interval) {
    memset(can_tx_data_, 0x00, 8);
    if (cmd_.list[0] == START) {
      can_tx_data_[0] = 0x91;
    } else if (cmd_.list[0] == STOP) {
      can_tx_data_[0] = 0x92;
    }
    HAL_CAN_AddTxMessage(hcan_, &can_tx_header_, can_tx_data_,
                         &can_tx_mail_box_);
    memmove(cmd_.list, (Cmd_e*)cmd_.list + 1, 4 * sizeof(Cmd_e));
    cmd_.list[cmd_.cnt - 1] = NO_CMD;
    cmd_.cnt--;
    cmd_.tick = HAL_GetTick();
  }
  // transmit control pack
  else {
    if (cmd_.cnt > 0) {
      tx_data_.torque = 0;
    } else {
      tx_data_.torque = motor_->intensity_float_;
    }
    tx_data_.duration = 0;
    can_tx_data_[0] = 0x93;
    uint32_t torque_int;
    memcpy(&torque_int, &tx_data_.torque, sizeof(float));
    memcpy(&can_tx_data_[1], &tx_data_.torque, sizeof(float));
    can_tx_data_[5] = tx_data_.duration & 0xff;
    can_tx_data_[6] = (tx_data_.duration >> 8) & 0xff;
    can_tx_data_[7] = (tx_data_.duration >> 16) & 0xff;
    HAL_CAN_AddTxMessage(hcan_, &can_tx_header_, can_tx_data_,
                         &can_tx_mail_box_);
  }
}

// Check CAN channel and CAN message header id
bool HTNCMotorDriver::canRxMsgCheck(CAN_HandleTypeDef* hcan,
                                    CAN_RxHeaderTypeDef rx_header) {
  return hcan == hcan_ && rx_header.StdId == master_id_;
}

// Receive feedback data message callback. Called in
// HAL_CAN_RxFifo0MsgPendingCallback()
void HTNCMotorDriver::canRxMsgCallback(CAN_HandleTypeDef* hcan,
                                       CAN_RxHeaderTypeDef rx_header,
                                       uint8_t rx_data[8]) {
  if (motor_->info_.type != Motor::HTNC || rx_data[0] != 0x93) {
    return;
  }

  // unpack feedback data
  // 反馈数据解包
  rx_data_.res = rx_data[1];
  rx_data_.temp = rx_data[2];
  uint16_t pos_int, speed_int, torque_int;
  pos_int = (rx_data[3]) | (rx_data[4] << 8);
  rx_data_.position = pos_int * 25.f / 65535.f - 12.5f;
  speed_int = (rx_data[5] << 4) | (rx_data[6] >> 4);
  rx_data_.speed = speed_int * 130.f / 4095.f - 65.f;
  torque_int = ((rx_data[6] & 0xf) << 4) | (rx_data[7]);
  rx_data_.torque = torque_int / 4095.f * 2.f - 1.f;

  // Encoder angle(deg)
  // 编码器角度
  motor_->motor_data_.ecd_angle = math::rad2deg(rx_data_.position);
  // Use incremental calculation to deals with encoder overflow and underflow
  // 增量计算处理编码器上下溢问题
  float delta =
      motor_->motor_data_.ecd_angle - motor_->motor_data_.last_ecd_angle;
  delta = math::degNormalize180(delta) / motor_->ratio_;
  motor_->motor_data_.angle += delta;  // deg
  // feedback rotational speed
  // 反馈转速
  motor_->motor_data_.rotate_speed =
      math::radps2dps(rx_data_.speed) / motor_->ratio_;  // dps
  // Update current
  // 更新转矩电流
  motor_->motor_data_.current = rx_data_.torque;
  // update encoder angle record
  // 更新编码器角度记录
  motor_->motor_data_.last_ecd_angle = motor_->motor_data_.ecd_angle;

  motor_->rxCallback();
}
