/**
 ******************************************************************************
 * @file    referee_comm.cpp/h
 * @brief   Referee communication(UART). 裁判系统通信(UART)
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef REFEREE_COMM_H
#define REFEREE_COMM_H

#include "base/common/connect.h"
#include "base/common/fifo_buffer.h"
#include "base/referee_comm/referee_protocol.h"
#include "usart.h"

#define REFEREE_TX_BUF_SIZE REFEREE_COMM_FRAME_MAX_SIZE
#define REFEREE_RX_BUF_SIZE REFEREE_COMM_FRAME_MAX_SIZE

// Referee communication class 裁判系统通信类
class RefereeComm {
 public:
  RefereeComm(UART_HandleTypeDef* huart = nullptr);

  // Init UART receive 初始化，打开UART接收
  void init(void);
  // Handle data, check connection 处理数据，检查连接状态
  void handle(void);

  // // Data transmit 数据发送
  void txMsg(void);

  // Data receive callback 接收中断
  void rxCallback(void);
  // UART端口校验
  bool uartCheck(UART_HandleTypeDef* huart) { return huart == huart_; }

  UART_HandleTypeDef* getHuart() const;

 public:
  Connect connect_;

  ext_game_status_t game_status_;      // 比赛状态数据
  ext_game_result_t game_result_;      // 比赛结果数据
  ext_game_robot_HP_t game_robot_HP_;  // 机器人血量数据
  // undefined dart_status, 0x0004
  // 人工智能挑战赛加成/惩罚区分布与潜伏模式状态
  ext_ICRA_buff_debuff_zone_and_lurk_status_t icra_status_;
  ext_event_data_t event_data_;  // 场地事件数据
  ext_supply_projectile_action_t supply_projectile_action_;  // 补给站动作标识
  ext_referee_warning_t referee_warning_;          // 裁判警告信息
  ext_dart_remaining_time_t dart_remaining_time_;  // 飞镖发射口倒计时

  ext_game_robot_status_t game_robot_status_;      // 比赛机器人状态
  ext_power_heat_data_t power_heat_data_;          // 实时功率热量数据
  ext_game_robot_pos_t game_robot_pos_;            // 机器人位置
  ext_buff_t game_buff_;                           // 机器人增益
  ext_aerial_robot_energy_t aerial_robot_energy_;  // 空中机器人能量状态
  ext_robot_hurt_t robot_hurt_;                    // 受到伤害状态
  ext_shoot_data_t shoot_data_;                    // 实时射击信息
  ext_bullet_remaining_t bullet_remaining_;        // 子弹剩余发射数
  ext_rfid_status_t rfid_status_;                  // 机器人RFID状态
  ext_dart_client_cmd_t dart_client_cmd_;  // 客户端飞镖指令数据

  // todo: interactive data(class)

 private:
  UART_HandleTypeDef* huart_;

  struct Tx_t {
    uint8_t buf[REFEREE_TX_BUF_SIZE];
    RefereeCommFrame_t frame;
    uint32_t pack_size;
  } tx_;

  struct Rx_t {
    Rx_t(void) : fifo(buf, REFEREE_RX_BUF_SIZE) {}

    uint8_t byte[1];
    uint8_t buf[REFEREE_RX_BUF_SIZE];
    FIFOBuffer fifo;
    RefereeCommFrame_t frame;
    uint32_t expect_size;
  } rx_;

  enum MessageUnpackStep_e {
    WAIT,
    DATA_LEN,
    SEQ,
    HEADER_CRC8,
    CMD_ID,
    DATA,
    PACK_CRC16,
    READ_DATA,
  } unpack_step_;

  enum UnpackError_e {
    NO_ERROR,
    DATA_LEN_OUTRANGE,
    HEADER_CRC_FAIL,
    ID_UNDEFINED,
    PACK_CRC_FAIL,
  } unpack_error_;
};

#endif  // REFEREE_COMM_H