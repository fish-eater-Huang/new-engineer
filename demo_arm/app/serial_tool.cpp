/**
 ******************************************************************************
 * @file    serial_tool.cpp/h
 * @brief   Serial tool message package. 串口工具(Serial tool)debug信息封装
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/serial_tool.h"
#include <stdio.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

extern IMU board_imu;
extern Arm arm;

// 数据发送封装，在robot.cpp中调用
void SerialStudio::handle(void) {
  // txIMUData(board_imu);
  // txMotorData(m1);
  // txGimbalData(gimbal);
  // txChassisData(chassis);
  txArmData(arm);

#ifdef DEBUG_USB
  // USB auto reconnect in suspended state
  // USB挂起状态下重新初始化自动重连
  if (hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED) {
    if (HAL_GetTick() - usb_init_tick > 100) {
      MX_USB_DEVICE_Init();
      usb_init_tick = HAL_GetTick();
    }
  }
#endif  // DEBUG_USB
}

// USB接收回调
void SerialStudio::usbRxCallback(uint8_t* buf, uint32_t len) {
  rx_.len = MIN(len, sizeof(rx_.buf));
  memcpy(rx_.buf, buf, rx_.len);
}

// UART接收回调
void SerialStudio::uartRxCallback(void) {
  // todo
}

// 数据打包发送
void SerialStudio::txData(float* data, uint16_t data_len) {
  tx_.len = sprintf((char*)tx_.buf, "%s", tx_.frame_start);
  for (int i = 0; i < data_len; i++) {
    tx_.len += sprintf((char*)tx_.buf + tx_.len, "%.3f,", data[i]);
  }
  tx_.len += sprintf((char*)tx_.buf + tx_.len, "%s", tx_.frame_end);

#ifdef DEBUG_USB
  CDC_Transmit_FS((uint8_t*)tx_.buf, tx_.len);
#endif  // DEBUG_USB
  if (huart_ != nullptr) {
    HAL_UART_Transmit_IT(huart_, (uint8_t*)tx_.buf, tx_.len);
  }
}

// IMU数据可视化/采集
void SerialStudio::txIMUData(IMU& imu) {
  sys_time_ = (float)HAL_GetTick() * 1e-3f;
  float txdata_pack[] = {
      sys_time_,       // %1, time
      imu.yaw(),       // %2, yaw
      imu.pitch(),     // %3, pitch
      imu.roll(),      // %4, roll
      imu.wxSensor(),  // %5, wx
      imu.wySensor(),  // %6, wy
      imu.wzSensor(),  // %7, wz
      imu.axSensor(),  // %8, ax(sensor)
      imu.aySensor(),  // %9, ay(sensor)
      imu.azSensor(),  // %10, az(sensor)
      imu.axWorld(),   // %11, ax(-g, world)
      imu.ayWorld(),   // %12, ay(-g, world)
      imu.azWorld(),   // %13, az(-g, world)
      imu.quat(0),     // %14, q0
      imu.quat(1),     // %15, q1
      imu.quat(2),     // %16, q2
      imu.quat(3),     // %17, q3
  };
  txData(txdata_pack, sizeof(txdata_pack) / sizeof(float));
}

// 电机数据可视化/采集
void SerialStudio::txMotorData(Motor& motor) {
  sys_time_ = (float)HAL_GetTick() * 1e-3f;
  float txdata_pack[] = {
      sys_time_,                       // %1
      motor.targetAngle(),             // %2
      motor.realAngle(),               // %3
      motor.targetSpeed(),             // %4
      motor.realSpeed(),               // %5
      motor.motor_data_.angle,         // %6
      motor.kf_data_.x[0],             // %7
      motor.motor_data_.rotate_speed,  // %8
      motor.kf_data_.x[1],             // %9
      (float)motor.intensity_,         // %10
      motor.motor_data_.current,       // %11
  };
  txData(txdata_pack, sizeof(txdata_pack) / sizeof(float));
}

// 机械臂数据可视化/采集
void SerialStudio::txArmData(Arm& arm) {
  sys_time_ = (float)HAL_GetTick() * 1e-3f;
  float txdata_pack[] = {
      sys_time_,                        // %1
      math::rad2deg(arm.ref_.q[0][0]),  // %2
      math::rad2deg(arm.fdb_.q[0][0]),  // %3
      math::rad2deg(arm.ref_.q[1][0]),  // %4
      math::rad2deg(arm.fdb_.q[1][0]),  // %5
      math::rad2deg(arm.ref_.q[2][0]),  // %6
      math::rad2deg(arm.fdb_.q[2][0]),  // %7
      math::rad2deg(arm.ref_.q[3][0]),  // %8
      math::rad2deg(arm.fdb_.q[3][0]),  // %9
      math::rad2deg(arm.ref_.q[4][0]),  // %10
      math::rad2deg(arm.fdb_.q[4][0]),  // %11
      math::rad2deg(arm.ref_.q[5][0]),  // %12
      math::rad2deg(arm.fdb_.q[5][0]),  // %13
      arm.ref_.x,                       // %14
      arm.fdb_.x,                       // %15
      arm.ref_.y,                       // %16
      arm.fdb_.y,                       // %17
      arm.ref_.z,                       // %18
      arm.fdb_.z,                       // %19
      math::rad2deg(arm.ref_.yaw),      // %20
      math::rad2deg(arm.fdb_.yaw),      // %21
      math::rad2deg(arm.ref_.pitch),    // %22
      math::rad2deg(arm.fdb_.pitch),    // %23
      math::rad2deg(arm.ref_.roll),     // %24
      math::rad2deg(arm.fdb_.roll),     // %25
  };
  txData(txdata_pack, sizeof(txdata_pack) / sizeof(float));
}
