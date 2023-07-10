/**
 ******************************************************************************
 * @file    imu_comm.cpp/h
 * @brief   IMU communication(CAN). IMU通信(CAN)
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef IMU_COMM_H
#define IMU_COMM_H

#include "base/common/connect.h"
#include "base/imu/imu.h"
#include "can.h"

class ImuComm {
  typedef struct ImuMsgPack {
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
  } ImuMsgPack_t;

 public:
  ImuComm(CAN_HandleTypeDef* hcan, IMU* imu1, IMU* imu2, IMU* imu3);

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

 private:
  CAN_HandleTypeDef* hcan_;
  IMU *imu1_, *imu2_, *imu3_;

  CAN_TxHeaderTypeDef can_tx_header_;
  uint8_t can_tx_data_[8];
  uint32_t can_tx_mail_box_;

  const uint16_t imu_comm_id_base_ = 0x100;

  ImuMsgPack_t imu_msg_[3];
};

#endif  // IMU_COMM_H
