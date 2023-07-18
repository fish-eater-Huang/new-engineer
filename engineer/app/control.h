/**
 ******************************************************************************
 * @file    control.cpp/h
 * @brief   Robot control design. 机器人控制设计（模式/键位）
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef CONTROL_H
#define CONTROL_H

#include "app/arm.h"
#include "app/arm_controller.h"
#include "app/gimbal.h"
#include "app/pump.h"
#include "base/remote/remote.h"

// 控制初始化
void controlInit(void);
// 控制主循环
void controlLoop(void);

// 机械臂任务类
class ArmTask {
  // 任务状态
  typedef enum State {
    MOVE,         // 行驶
    EXCHANGE,     // 兑换
    PICK_NORMAL,  // 取矿(资源岛)
    PICK_LOW,     // 取矿(地面)/障碍块/救援
    PICK_HIGH,    // 空接
    DEPOSIT_1,    // 存矿1(左)
    DEPOSIT_2,    // 存矿2(右)
    WITHDRAW_1,   // 出矿1(左)
    WITHDRAW_2,   // 出矿2(右)
  } State_e;

  // 机械臂位姿参数
  typedef struct Pose {
    float x, y, z;
    float yaw, pitch, roll;

    Pose(float arg_x, float arg_y, float arg_z, float arg_yaw, float arg_pitch,
         float arg_roll)
        : x(arg_x),
          y(arg_y),
          z(arg_z),
          yaw(arg_yaw),
          pitch(arg_pitch),
          roll(arg_roll) {}
  } Pose_t;

 public:
  // 构造函数
  ArmTask(RC* rc, Arm* arm, ArmController* controller, Pump* pump_e,
          Pump* pump_0);

  // 遥控器控制
  void rcCtrl(void);
  // 控制器控制
  void controllerCtrl(void);

  // 切换任务状态
  void switchState(ArmTask::State_e state);
  // 暂停任务
  void pause(void);

  // 任务处理
  void handle(void);

 public:
  RC* rc_;                     // 遥控器指针
  Arm* arm_;                   // 机械臂指针
  ArmController* controller_;  // 控制器指针
  ArmGimbal* gimbal_;          // 云台指针
  Pump *pump_e_, *pump_0_;     // 气泵指针

  // 任务状态
  State_e state_;
  // 暂停标记
  bool pause_;

  // 机械臂控制方式
  enum ControlMethod_e {
    MANIPULATION_FPV,  // 第一人称工作空间控制
    MANIPULATION_TPV,  // 第三人称工作空间控制
    JOINT,             // 关节控制
    JOINT_FREE,        // 无限制关节控制，用于手动初始化
  } method_;

  // 步骤切换时间
  uint32_t tick_next_step;

  // 控制参数
  struct ControlParam_t {
    // 遥控器控制参数
    struct RCCtrl_t {
      float position_rate = 1e-6f;
      float arm_direction_rate = 3e-6f;
      float arm_joint_rate = 3e-6f;
    } rc;
    // 控制器控制参数
    struct Controller_t {
      float x_rate = 1;
      float y_rate = 1;
      float z_rate = 1;
    } controller;
    // 轨迹参数
    struct Traj_t {
      float speed = 0.5;
      float rotate_speed = PI;
    } traj;
    // 速度比例
    struct Ratio_t {
      float fast = 1.2;
      float slow = 0.5;
    } ratio;
  } ctrl_;

  // 兑换
  struct Exchange_t {
    // 步骤
    enum Step_e {
      TRAJ_DEFAULT,  // 移动至默认位置
      LOCATE,        // 定位
      PUMP_E_OFF,    // 关闭机械臂气泵
      TRAJ_ROTATE,   // 末端转90°
    } step;
  } exchange_;

  // 取矿
  struct Pick_t {
    // 步骤
    enum Step_e {
      TRAJ_DEFAULT,      // 移动至默认位置(三连23矿跳过)
      LOCATE,            // 定位
      PUMP_E_ON,         // 开启机械臂气泵
      TRAJ_PICK_UPWARD,  // 升高(空接跳过该步)
    } step;
    // 定位位姿
    Pose_t start_pose;
    // 默认位姿参数
    const Pose_t normal_default = Pose(0.235, 0, 0.15, 0, 0, 0);
    const Pose_t high_default = Pose(0.5, 0, 0.3, 0, -PI * 0.5, 0);
    const Pose_t low_default = Pose(0.235, 0, 0, 0, PI * 0.5, 0);
    // 取矿升高距离
    const float pick_upward = 0.3;
  } pick_;

  // 三连取矿
  struct TriplePick_t {
    // 三连状态
    bool state;
    // 步骤
    enum Step_e {
      PICK_1,     // 取1
      DEPOSIT_1,  // 存1
      PICK_2,     // 取2
      DEPOSIT_2,  // 存2
      PICK_3,     // 取3
      DEPOSIT_3,  // 存3
    } step;
    // 矿石间隔
    const float mine_dist = 0.27;
  } triple_pick_;

  // 存矿
  struct Deposit_t {
    // 步骤
    enum Step_e {
      PUMP_E_ON,              // 开启机械臂气泵
      TRAJ_RELAY_1,           // 移动至中间点1(x+)
      TRAJ_RELAY_2,           // 移动至中间点2(避免干涉)
      TRAJ_DEPOSIT_ABOVE,     // 移动至存矿上方
      TRAJ_DEPOSIT_DOWNWARD,  // 降低
      PUMP_0_ON,              // 开启存矿气泵
      PUMP_E_OFF,             // 关闭机械臂气泵
      TRAJ_DEPOSIT_UPWARD,    // 升高
    } step;
    // 中间点1(正前方)
    const Pose_t relay1 = Pose(0.235, 0, 0.3, 0, 0, 0);
    // 中间点2(左右，避免干涉)
    const Pose_t relay2_1 = Pose(0, 0.1, 0.4, 0, PI * 0.5, 0);
    const Pose_t relay2_2 = Pose(0, -0.1, 0.4, 0, PI * 0.5, 0);
    // 存矿吸盘上方
    const Pose_t deposite_1_above = Pose(-0.3, 0.1, 0.22, 0, PI * 0.5, 0);
    const Pose_t deposite_2_above = Pose(-0.3, -0.1, 0.22, 0, PI * 0.5, 0);
    // 存矿吸盘
    const Pose_t deposite_1 = Pose(-0.3, 0.1, 0.12, 0, PI * 0.5, 0);
    const Pose_t deposite_2 = Pose(-0.3, -0.1, 0.12, 0, PI * 0.5, 0);
  } deposite_;

  // 出矿
  struct Withdraw_t {
    // 步骤
    enum Step_e {
      TRAJ_DEPOSIT_ABOVE,     // 移动至存矿上方
      TRAJ_DEPOSIT_DOWNWARD,  // 降低
      PUMP_E_ON,              // 开启机械臂气泵
      PUMP_0_OFF,             // 关闭存矿气泵
      TRAJ_DEPOSIT_UPWARD,    // 升高
      TRAJ_RELAY_2,           // 移动至中间点2
      TRAJ_EXCHANGE,          // 移动至兑换默认位姿
    } step;
  } withdraw_;
};

#endif  // CONTROL_H
