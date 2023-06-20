/**
 ******************************************************************************
 * @file    board_comm.cpp/h
 * @brief   Board communication(CAN). 板间通信(CAN)
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/board_comm.h"
#include <string.h>
#include "app/imu_monitor.h"

extern uint8_t board_id;

const uint32_t board_comm_timeout = 1000;

BoardComm::BoardComm(CAN_HandleTypeDef* hcan)
    : connect_(board_comm_timeout), hcan_(hcan) {}

void BoardComm::handle(void) {
  connect_.check();
}

// Transmit data
// 发送数据
void BoardComm::canTxMsg(void) {
  can_tx_header_.IDE = CAN_ID_STD;
  can_tx_header_.RTR = CAN_RTR_DATA;
  can_tx_header_.DLC = 8;
  can_tx_header_.StdId = board_id;
  memcpy(can_tx_data_, &imu_msg_[board_id - 1], sizeof(ImuMsgPack_t));
  // transmit
  HAL_CAN_AddTxMessage(hcan_, &can_tx_header_, can_tx_data_, &can_tx_mail_box_);
}

// Check CAN channel and id of received CAN message
// 校验接收信息的CAN通道和ID
bool BoardComm::canRxMsgCheck(CAN_HandleTypeDef* hcan,
                              CAN_RxHeaderTypeDef rx_header) {
  return hcan == hcan_ && (rx_header.StdId >= board_comm_id_base_ + 1 ||
                           rx_header.StdId <= board_comm_id_base_ + 3);
}

// Receive feedback data message callback. Called in
// HAL_CAN_RxFifo0MsgPendingCallback()
// 信息接收回调，在HAL_CAN_RxFifo0MsgPendingCallback中调用
void BoardComm::canRxMsgCallback(CAN_HandleTypeDef* hcan,
                                 CAN_RxHeaderTypeDef rx_header,
                                 uint8_t rx_data[8]) {
  uint8_t rx_id = rx_header.StdId - board_comm_id_base_;
  memcpy(&imu_msg_[rx_id - 1], rx_data, sizeof(ImuMsgPack_t));
  controller_imu[rx_id - 1].yaw() =
      (float)imu_msg_[rx_id - 1].yaw * 180.f / 32767.f;
  controller_imu[rx_id - 1].pitch() =
      (float)imu_msg_[rx_id - 1].pitch * 180.f / 32767.f;
  controller_imu[rx_id - 1].roll() =
      (float)imu_msg_[rx_id - 1].roll * 180.f / 32767.f;
  connect_.refresh();
}
