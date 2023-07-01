/**
 ******************************************************************************
 * @file    board_comm.cpp/h
 * @brief   Board communication(CAN). 板间通信(CAN)
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef BOARD_COMM_H
#define BOARD_COMM_H

#include "base/common/connect.h"
#include "can.h"

class BoardComm {
  typedef struct ImuMsgPack {
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
  } ImuMsgPack_t;

 public:
  BoardComm(CAN_HandleTypeDef* hcan);

  void handle(void);

  // Transmit data
  // 发送数据
  void canTxMsg(void);

  // Check CAN channel and id of received CAN message
  // 校验接收信息的CAN通道和ID
  bool canRxMsgCheck(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header);

  // Receive feedback data message callback. Called in
  // HAL_CAN_RxFifo0MsgPendingCallback()
  // 信息接收回调，在HAL_CAN_RxFifo0MsgPendingCallback中调用
  void canRxMsgCallback(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header,
                        uint8_t rx_data[8]);

 public:
  Connect imu1_connect_, imu2_connect_, imu3_connect_;
  ImuMsgPack_t imu_msg_[3];

 private:
  CAN_HandleTypeDef* hcan_;
  CAN_TxHeaderTypeDef can_tx_header_;
  uint8_t can_tx_data_[8];
  uint32_t can_tx_mail_box_;

  const uint16_t board_comm_id_base_ = 0x100;
};

#endif  // BOARD_COMM_H
