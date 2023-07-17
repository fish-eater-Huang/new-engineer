/**
 ******************************************************************************
 * @file    arm_controller.cpp/h
 * @brief   6-DOF arm controller program. 6轴机械臂控制器程序
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include "usart.h"

#include "base/common/connect.h"
#include "base/common/fifo_buffer.h"
#include "base/common/filter.h"
#include "base/imu/imu.h"
#include "base/referee_comm/referee_protocol.h"
#include "base/robotics/robotics.h"

#define CONTROLLER_TX_BUF_SIZE 64
#define CONTROLLER_RX_BUF_SIZE 64

const uint16_t controller_cmd_id = 0x302;

namespace imucomm {
// IMU数据类型转换，int16->float
float int16_2_float(const int16_t& deg);
// IMU数据类型转换，float->int16
int16_t float_2_int16(const float& deg);
}  // namespace imucomm

// 控制器通信类
class ControllerComm {
  typedef struct ImuStatus {
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
  } ImuStatus_t;

 public:
  ControllerComm(UART_HandleTypeDef* huart = nullptr);

  // 初始化，打开UART接收
  void init(void);
  // 处理数据，检查连接状态
  void handle(void);

  // 数据发送
  void txMsg(void);
  // 接收中断
  void rxCallback(void);
  // UART端口校验
  bool uartCheck(UART_HandleTypeDef* huart) { return huart == huart_; }

 public:
  // 连接状态
  Connect connect_;

  // 发送数据
  struct TxData_t {
    uint8_t controller_state;
    uint8_t imu_connect[3];
    ImuStatus_t imu[3];
    uint8_t reserve[8];
  } __packed tx_data_;

  // 接收数据
  struct RxData_t {
    uint8_t controller_state;
    uint8_t imu_connect[3];
    ImuStatus_t imu[3];
    uint8_t reserve[8];
  } __packed rx_data_;

 private:
  // UART指针
  UART_HandleTypeDef* huart_;

  // 发送相关结构体
  struct Tx_t {
    uint8_t buf[CONTROLLER_TX_BUF_SIZE];
    RefereeCommFrame_t frame;
    uint32_t pack_size;
  } tx_;

  // 接收相关结构体
  struct Rx_t {
    Rx_t(void) : fifo(buf, CONTROLLER_RX_BUF_SIZE) {}

    uint8_t byte[1];
    uint8_t buf[CONTROLLER_RX_BUF_SIZE];
    FIFOBuffer fifo;
    RefereeCommFrame_t frame;
    uint32_t expect_size;
  } rx_;

  // 解包步骤
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

  // 解包错误
  enum UnpackError_e {
    NO_ERROR,
    DATA_LEN_OUTRANGE,
    HEADER_CRC_FAIL,
    ID_UNDEFINED,
    PACK_CRC_FAIL,
  } unpack_error_;
};

// 机械臂控制器(3*imu)
class ArmController {
 public:
  // 构造函数
  ArmController(ControllerComm* comm, IMU imu[3]);

  // 设置偏置值
  void setOffset(float dx, float dy, float dz);

  // 增量设置偏置值
  void addOffset(float dx, float dy, float dz);

  // 设置yaw零点
  void setYawZero(void);

  // 控制器处理函数
  void handle(void);

 public:
  // imu指针
  IMU* imu_[3];
  // 控制器通信指针
  ControllerComm* comm_;

  // 控制器状态
  bool state_;

  // 目标状态
  struct Ref_t {
    Matrixf<4, 4> T;
    float x, y, z;           // m
    float yaw, pitch, roll;  // rad
    // 目标状态滤波
    LowPassFilter x_filter, y_filter, z_filter;
    LowPassFilter yaw_filter, pitch_filter, roll_filter;

    Ref_t(const float& k)
        : x_filter(k),
          y_filter(k),
          z_filter(k),
          yaw_filter(k),
          pitch_filter(k),
          roll_filter(k) {}
  } ref_;

  // 未处理状态
  struct Raw_t {
    float x, y, z;           // m
    float yaw, pitch, roll;  // rad
  } raw_;

  // 状态偏置
  struct Offset_t {
    float x, y, z;  // m
    float yaw[3];   // deg
  } offset_;

  // 参数
  struct Param_t {
    float l[2] = {0.3f, 0.27f};
  } param_;
};

#endif  // ARM_CONTROLLER_H
