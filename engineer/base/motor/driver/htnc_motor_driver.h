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

#ifndef HTNC_MOTOR_DRIVER_H
#define HTNC_MOTOR_DRIVER_H

#include "base/motor/motor.h"
#include "can.h"

class HTNCMotorDriver {
 public:
  // motor mode command
  typedef enum Cmd {
    NO_CMD = 0,
    START = 1,
    STOP = 2,
  } Cmd_e;

 public:
  // master id: motor->board
  // slave id: board->motor
  HTNCMotorDriver(Motor* motor, CAN_HandleTypeDef* hcan, uint32_t master_id,
                  uint32_t slave_id);

  // set motor mode command
  bool setCmd(HTNCMotorDriver::Cmd_e cmd);

  // Transmit CAN message
  void canTxMsg(void);

  // Check CAN channel and CAN message header id
  bool canRxMsgCheck(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header);

  // Receive feedback data message callback. Called in
  // HAL_CAN_RxFifo0MsgPendingCallback()
  void canRxMsgCallback(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef rx_header,
                        uint8_t rx_data[8]);

 public:
  // motor pointer
  Motor* motor_;

  // transmit data pack
  struct TxData {
    float torque;
    uint32_t duration;
  } tx_data_;

  // receive data pack
  struct RxData {
    uint8_t res;
    uint8_t temp;    // degree
    float position;  // rad
    float speed;     // rad/s
    float torque;    // N·m
  } rx_data_;

 private:
  CAN_HandleTypeDef* hcan_;
  uint32_t master_id_;  // motor to board
  uint32_t slave_id_;   // board to motor

  // can transmit
  CAN_TxHeaderTypeDef can_tx_header_;
  uint8_t can_tx_data_[8];
  uint32_t can_tx_mail_box_;

  // motor command
  struct Command_t {
    uint32_t tick;
    const uint32_t interval = 100;
    Cmd_e list[5] = {HTNCMotorDriver::Cmd_e::NO_CMD};
    uint8_t cnt = 0;
  } cmd_;
};

#endif  // HTNC_MOTOR_DRIVER_H
