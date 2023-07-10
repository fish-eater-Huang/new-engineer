/**
 ******************************************************************************
 * @file    arm_controller.cpp/h
 * @brief   6-DOF arm controller program. 6轴机械臂控制器程序
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/arm_controller.h"
#include "base/common/crc.h"

// IMU数据类型转换，int16->float
float imucomm::int16_2_float(const int16_t& deg) {
  return (deg * 180.f / 32767.f);
}

// IMU数据类型转换，float->int16
int16_t imucomm::float_2_int16(const float& deg) {
  return (deg / 180.f * 32767.f);
}

ControllerComm::ControllerComm(UART_HandleTypeDef* huart)
    : huart_(huart), connect_(1000), unpack_step_(WAIT) {}

// 初始化，打开UART接收
void ControllerComm::init(void) {
  if (huart_ != nullptr) {
    HAL_UART_Receive_IT(huart_, rx_.byte, 1);
  }
}

// 处理数据，检查连接状态
void ControllerComm::handle(void) {
  connect_.check();
  txMsg();
}

// 数据发送
void ControllerComm::txMsg(void) {
  // header
  tx_.frame.header.sof = referee_comm_sof;
  tx_.frame.header.data_len = sizeof(tx_data_);
  tx_.frame.header.seq++;
  memcpy(tx_.buf, &tx_.frame.header, sizeof(tx_.frame.header) - 1);
  CRC8_Append(tx_.buf, sizeof(tx_.frame.header));
  tx_.pack_size = sizeof(tx_.frame.header);

  // cmd_id
  tx_.frame.cmd_id = controller_cmd_id;
  memcpy(tx_.buf + tx_.pack_size, &tx_.frame.cmd_id, sizeof(tx_.frame.cmd_id));
  tx_.pack_size += sizeof(tx_.frame.cmd_id);

  // data
  memcpy(tx_.buf + tx_.pack_size, &tx_data_, sizeof(tx_data_));
  tx_.pack_size += sizeof(tx_data_);

  // tail
  tx_.pack_size += sizeof(tx_.frame.crc16);
  CRC16_Append(tx_.buf, tx_.pack_size);

  // UART transmit
  if (huart_ != nullptr) {
    HAL_UART_Transmit_IT(huart_, tx_.buf, tx_.pack_size);
  }
}

// 接收中断
void ControllerComm::rxCallback(void) {
  // 将接收到的1byte数据存入fifo缓冲区
  rx_.fifo.append(rx_.byte, 1);

  // 数据解包
  // 等待阶段
  if (unpack_step_ == WAIT) {
    uint32_t sof_index = rx_.fifo.find(referee_comm_sof);
    if (sof_index != rx_.fifo.size()) {
      // 检测到SOF标志，清除缓冲区无效数据，进入data_len阶段
      rx_.fifo.remove(sof_index);
      rx_.expect_size =
          sizeof(rx_.frame.header.sof) + sizeof(rx_.frame.header.data_len);
      unpack_step_ = DATA_LEN;
    } else {
      // 未检测到SOF标志，清空缓冲区
      rx_.fifo.clear();
    }
  }
  // data_len阶段
  if (unpack_step_ == DATA_LEN) {
    if (rx_.fifo.size() >= rx_.expect_size) {
      // 记录data_len，进入seq阶段
      memcpy(&rx_.frame.header.data_len, rx_.buf + sizeof(rx_.frame.header.sof),
             sizeof(rx_.frame.header.data_len));
      if (rx_.frame.header.data_len + sizeof(rx_.frame) > rx_.fifo.maxSize()) {
        // 数据长度错误
        rx_.fifo.clear();
        unpack_error_ = DATA_LEN_OUTRANGE;
        unpack_step_ = WAIT;
      } else {
        rx_.expect_size += sizeof(rx_.frame.header.seq);
        unpack_step_ = SEQ;
      }
    }
  }
  // seq阶段
  if (unpack_step_ == SEQ) {
    if (rx_.fifo.size() >= rx_.expect_size) {
      // 记录seq，进入帧头CRC8校验阶段
      memcpy(&rx_.frame.header.seq,
             rx_.buf + sizeof(rx_.frame.header.sof) +
                 sizeof(rx_.frame.header.data_len),
             sizeof(rx_.frame.header.seq));
      rx_.expect_size += sizeof(rx_.frame.header.crc8);
      unpack_step_ = HEADER_CRC8;
    }
  }
  // 帧头CRC8校验
  if (unpack_step_ == HEADER_CRC8) {
    if (rx_.fifo.size() >= rx_.expect_size) {
      rx_.frame.header.crc8 = CRC8_Calc(rx_.buf, rx_.expect_size);
      if (CRC8_Verify(rx_.buf, rx_.expect_size)) {
        // CRC8校验成功
        rx_.expect_size += sizeof(rx_.frame.cmd_id);
        unpack_step_ = CMD_ID;
      } else {
        // CRC8校验失败
        rx_.fifo.clear();
        unpack_error_ = HEADER_CRC_FAIL;
        unpack_step_ = WAIT;
      }
    }
  }
  // cmd id
  if (unpack_step_ == CMD_ID) {
    if (rx_.fifo.size() >= rx_.expect_size) {
      // 记录cmd_id，进入data阶段
      memcpy(&rx_.frame.cmd_id, rx_.buf + sizeof(rx_.frame.header),
             sizeof(rx_.frame.cmd_id));
      rx_.expect_size += rx_.frame.header.data_len;
      unpack_step_ = DATA;
    }
  }
  // data阶段，接收数据量足够后进入帧尾校验阶段
  if (unpack_step_ == DATA) {
    if (rx_.fifo.size() >= rx_.expect_size) {
      // 进入tail阶段
      rx_.expect_size += sizeof(rx_.frame.crc16);
      unpack_step_ = PACK_CRC16;
    }
  }
  // 帧尾CRC16校验阶段，接收完帧尾后进行校验
  if (unpack_step_ == PACK_CRC16) {
    if (rx_.fifo.size() >= rx_.expect_size) {
      rx_.frame.crc16 = CRC16_Calc(rx_.buf, rx_.expect_size);
      if (CRC16_Verify(rx_.buf, rx_.expect_size)) {
        // CRC校验成功
        unpack_step_ = READ_DATA;
      } else {
        // CRC校验失败
        rx_.fifo.clear();
        unpack_error_ = PACK_CRC_FAIL;
        unpack_step_ = WAIT;
      }
    }
  }

  // read_data阶段，从缓冲区读取数据
  if (unpack_step_ == READ_DATA) {
    const static uint8_t data_offset =
        sizeof(rx_.frame.header) + sizeof(rx_.frame.cmd_id);
    unpack_error_ = NO_ERROR;
    if (rx_.frame.cmd_id == controller_cmd_id) {
      memcpy(&rx_data_, rx_.buf + data_offset, sizeof(rx_data_));
    } else {
      unpack_error_ = ID_UNDEFINED;
    }
    rx_.fifo.remove(rx_.expect_size);  // 从缓冲区移除该帧数据
    unpack_step_ = WAIT;               // 重置解包状态
    // 刷新连接状态
    if (unpack_error_ == NO_ERROR) {
      connect_.refresh();
    }
  }

  // 重新打开串口接收
  if (huart_ != nullptr) {
    HAL_UART_Receive_IT(huart_, rx_.byte, 1);
  }
}

// 机械臂控制器构造函数
ArmController::ArmController(ControllerComm* comm, IMU imu[3])
    : comm_(comm), ref_(0.05f) {
  imu_[0] = &imu[0];
  imu_[1] = &imu[1];
  imu_[2] = &imu[2];
}

// 设置机械臂控制器位置偏置值
void ArmController::setOffset(float dx, float dy, float dz) {
  offset_.x = dx;
  offset_.y = dy;
  offset_.z = dz;
  ref_.x_filter.reset(raw_.x + offset_.x);
  ref_.y_filter.reset(raw_.y + offset_.y);
  ref_.z_filter.reset(raw_.z + offset_.z);
}

// 设置yaw零点
void ArmController::setYawZero(void) {
  for (int i = 0; i < 3; i++) {
    offset_.yaw[i] = imu_[i]->yaw();
  }
}

// 机械臂控制器处理函数
void ArmController::handle(void) {
  // 检测控制器连接状态
  if (comm_->connect_.check()) {
    state_ = comm_->rx_data_.controller_state;
    // 更新imu数据
    for (int i = 0; i < 3; i++) {
      if (comm_->rx_data_.imu_connect[i]) {
        imu_[i]->yaw() = imucomm::int16_2_float(comm_->rx_data_.imu[i].yaw);
        imu_[i]->pitch() = imucomm::int16_2_float(comm_->rx_data_.imu[i].pitch);
        imu_[i]->roll() = imucomm::int16_2_float(comm_->rx_data_.imu[i].roll);
      }
    }
  } else {
    state_ = false;
  }

  // 目标姿态
  raw_.yaw = math::deg2rad(imu_[2]->yaw());
  raw_.pitch = math::deg2rad(imu_[2]->pitch());
  raw_.roll = math::deg2rad(imu_[2]->roll());
  ref_.yaw = raw_.yaw;
  ref_.pitch = raw_.pitch;
  ref_.roll = raw_.roll;

  // 目标位置
  Matrixf<3, 1> p11 = matrixf::zeros<3, 1>();
  p11[0][0] = param_.l[0];  // [l1;0;0]
  Matrixf<3, 1> p22 = matrixf::zeros<3, 1>();
  p22[0][0] = param_.l[1];  // [l2;0;0]
  float rpy1[3] = {math::deg2rad(imu_[0]->yaw()),
                   math::deg2rad(imu_[0]->pitch()),
                   math::deg2rad(imu_[0]->roll())};
  float rpy2[3] = {math::deg2rad(imu_[1]->yaw()),
                   math::deg2rad(imu_[1]->pitch()),
                   math::deg2rad(imu_[1]->roll())};
  Matrixf<3, 3> R01 = robotics::rpy2r(rpy1);
  Matrixf<3, 3> R02 = robotics::rpy2r(rpy2);
  Matrixf<3, 1> p = R01 * p11 + R02 * p22;
  raw_.x = p[0][0];
  raw_.y = p[1][0];
  raw_.z = p[2][0];

  // 目标位置+偏置
  ref_.x = raw_.x + offset_.x;
  ref_.y = raw_.y + offset_.y;
  ref_.z = raw_.z + offset_.z;

  // 目标状态T矩阵
  float ref_p[3] = {ref_.x, ref_.y, ref_.z};
  float ref_rpy[3] = {ref_.yaw, ref_.pitch, ref_.roll};
  ref_.T = robotics::rp2t(robotics::rpy2r(ref_rpy), ref_p);
}
