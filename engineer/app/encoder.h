/**
 ******************************************************************************
 * @file    encoder.cpp/h
 * @brief   encoder communication(CAN). 编码器驱动板通信(CAN)
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "base/common/connect.h"
#include "can.h"

class EncoderComm {
 public:
  EncoderComm(CAN_HandleTypeDef* hcan);

  void handle(void);

  // Check CAN channel and id of received CAN message
  // 校验接收信息的CAN通道和ID
  bool canRxMsgCheck(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header);

  // Receive feedback data message callback. Called in
  // HAL_CAN_RxFifo0MsgPendingCallback()
  // 信息接收回调，在HAL_CAN_RxFifo0MsgPendingCallback中调用
  void canRxMsgCallback(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header,
                        uint8_t rx_data[8]);

 public:
  Connect connect_;

  // 接收数据
  struct RxData_t {
    uint16_t ecd[3];
    uint16_t reserve;
  } rx_data_;

  // 编码器角度
  float deg_[3];

 private:
  CAN_HandleTypeDef* hcan_;
  const uint16_t id_ = 0x221;
};

#endif  // ENCODER_H
