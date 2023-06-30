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

#ifndef HT_MOTOR_DRIVER_H
#define HT_MOTOR_DRIVER_H

#include "can.h"

#include "base/motor/motor.h"

namespace ht04 {

// position limit(rad)
const float p_min = -95.5f;
const float p_max = 95.5f;
// velocity limit(rad/s)
const float v_min = -45;
const float v_max = 45;
// PD param limit
const float kp_min = 0;
const float kp_max = 500;
const float kv_min = 0;
const float kv_max = 5;
// feedforward torque limit(N·m)
const float t_ff_min = -40;
const float t_ff_max = 40;
// feedback torque limit(N·m)
const float t_min = -40;
const float t_max = 40;

// motor mode command
typedef enum CmdType {
  NO_CMD = 0,
  MOTOR_MODE = 1,
  RESET_MODE = 2,
  ZERO_POSITION = 3,
} CmdType_e;

// feedback data
typedef struct Feedback {
  uint8_t id;
  float position;  // rad
  float velocity;  // rad/s
  float torq;      // N·m
} Feedback_t;

};  // namespace ht04

class HT04Driver {
 public:
  HT04Driver(Motor* motor, CAN_HandleTypeDef* hcan, uint32_t id)
      : motor_(motor), hcan_(hcan), id_(id) {}

  // set motor mode command
  bool setCmd(ht04::CmdType_e cmd);

  // set motor control parameter
  // p: target position(rad)
  // v: target velocity(rad/s)
  // kp/kv: control gain
  // t: feedforward torque
  // T = kp*(p-p_fdb)+kv*(v-v_fdb)+t
  void setControlParam(float p, float v, float kp, float kv, float t);

  // set torque
  void setTorque(const float& torque);

  // can transmit message
  void canTxMsg(void);

  // Check CAN channel and CAN message header id
  // 校验CAN通道和ID
  bool canRxMsgCheck(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header);

  // Receive feedback data message callback. Called in
  // HAL_CAN_RxFifo0MsgPendingCallback()
  // 电调反馈信息接受回调，在HAL_CAN_RxFifo0MsgPendingCallback中调用
  void canRxMsgCallback(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header,
                        uint8_t rx_data[8]);

 public:
  Motor* motor_;

 private:
  CAN_HandleTypeDef* hcan_;
  uint32_t id_;

  // transmit
  CAN_TxHeaderTypeDef can_tx_header_;
  uint8_t can_tx_data_[8];
  uint32_t can_tx_mail_box_;

  uint32_t cmd_tick_;
  const uint32_t cmd_interval_ = 100;
  ht04::CmdType_e cmd_list_[5] = {ht04::NO_CMD};
  uint8_t cmd_cnt_ = 0;

  struct TxParam {
    uint16_t p;
    uint16_t v;
    uint16_t kp;
    uint16_t kv;
    uint16_t t_ff;
  } tx_param_;

  // receive
  ht04::Feedback_t raw_data_;
};

#endif  // HT_MOTOR_DRIVER_H
