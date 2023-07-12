/**
 ******************************************************************************
 * @file    encoder.cpp/h
 * @brief   encoder communication(CAN). 编码器驱动板通信(CAN)
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/encoder.h"
#include <string.h>

EncoderComm::EncoderComm(CAN_HandleTypeDef* hcan)
    : hcan_(hcan), connect_(1000) {
  connect_.check();
}

void EncoderComm::handle(void) {}

// Check CAN channel and id of received CAN message
// 校验接收信息的CAN通道和ID
bool EncoderComm::canRxMsgCheck(CAN_HandleTypeDef* hcan,
                                CAN_RxHeaderTypeDef rx_header) {
  return hcan == hcan_ && (rx_header.StdId == id_);
}

// Receive feedback data message callback. Called in
// HAL_CAN_RxFifo0MsgPendingCallback()
// 信息接收回调，在HAL_CAN_RxFifo0MsgPendingCallback中调用
void EncoderComm::canRxMsgCallback(CAN_HandleTypeDef* hcan,
                                   CAN_RxHeaderTypeDef rx_header,
                                   uint8_t rx_data[8]) {
  memcpy(&rx_data_, rx_data, sizeof(rx_data_));
  for (int i = 0; i < 3; i++) {
    deg_[i] = (float)rx_data_.ecd[i] * 360.f / 65535.f;
  }
  connect_.refresh();
}
