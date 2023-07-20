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

// 机械臂位姿参数
typedef struct Pose {
  float x, y, z;
  float yaw, pitch, roll;

  Pose(float arg_x = 0, float arg_y = 0, float arg_z = 0, float arg_yaw = 0,
       float arg_pitch = 0, float arg_roll = 0)
      : x(arg_x),
        y(arg_y),
        z(arg_z),
        yaw(arg_yaw),
        pitch(arg_pitch),
        roll(arg_roll) {}
} Pose_t;

// 机械臂任务类
class ArmTask {
 public:
  // 任务类别
  typedef enum Mode {
    MOVE,         // 行驶
    PICK_NORMAL,  // 取矿(资源岛)
    PICK_LOW,     // 取矿(地面)/障碍块/救援
    PICK_HIGH,    // 取矿(空接)
    DEPOSIT_0,    // 存矿0(左)
    DEPOSIT_1,    // 存矿1(右)
    WITHDRAW_0,   // 出矿0(左)
    WITHDRAW_1,   // 出矿1(右)
    EXCHANGE,     // 兑换
  } Mode_e;

  // 任务状态
  typedef enum TaskState {
    IDLE,
    WORKING,
  } TaskState_e;

  // 取矿方式
  typedef enum PickMethod {
    SINGLE,
    TRIPLE,
  } PickMethod_e;

 public:
  // 构造函数
  ArmTask(void) {}

  // 初始化
  void reset(void);

  // 切换任务模式
  void switchMode(ArmTask::Mode_e mode);

  // 任务处理
  void handle(void);

  // 开始取矿
  void startPick(PickMethod_e method);

  // 开始兑换
  void startExchange(void);

  // 重置存矿计数
  void resetDepositCnt(void);

  // 存矿气泵电机处理
  void depositPumpHandle(void);

  // 三连取矿处理
  void triplePickHandle(void);

 public:
  // 任务状态
  Mode_e mode_;

  // 步骤切换时间
  uint32_t finish_tick_;

  // 移动
  struct Move_t {
    // 状态
    TaskState_e state;
    uint32_t* finish_tick;
    // 步骤
    enum Step_e {
      PREPARE,       // 准备
      TRAJ_RELAY,    // 机械臂中间点
      TRAJ_RETRACT,  // 机械臂收回
    } step;
    // 机械臂中间点关节角度
    float q_relay[6] = {math::deg2rad(0),     math::deg2rad(-135.0f),
                        math::deg2rad(60.0f), math::deg2rad(0),
                        math::deg2rad(0),     math::deg2rad(0)};
    // 机械臂收回关节角度
    float q_retract[6] = {math::deg2rad(0),     math::deg2rad(-165.0f),
                          math::deg2rad(75.0f), math::deg2rad(0),
                          math::deg2rad(0),     math::deg2rad(0)};

    // 处理函数
    void handle(void);
  } move_;

  // 取矿
  struct Pick_t {
    // 状态
    TaskState_e state;
    uint32_t* finish_tick;
    // 步骤
    enum Step_e {
      PREPARE,           // 准备
      TRAJ_DEFAULT,      // 移动至默认位置(三连23矿跳过)
      LOCATE,            // 定位
      PUMP_E_ON,         // 开启机械臂气泵
      TRAJ_PICK_UPWARD,  // 升高
    } step;
    // 取矿模式，0-normal，1-high，2-low
    uint8_t type;
    // 默认位姿参数
    Pose_t default_pose[3] = {
        Pose(0.235, 0, 0.15, 0, 0, 0),
        Pose(0.5, 0, 0.5, 0, -PI * 0.5, 0),
        Pose(0.3, 0, -0.2, 0, PI * 0.5, 0),
    };
    // 定位位姿
    Pose_t start_pose;
    // 取矿升高距离
    Pose_t pick_up_pose;
    float pick_up_dist = 0.3;

    // 处理函数
    void handle(void);
  } pick_;

  // 存矿
  struct Deposit_t {
    // 状态
    TaskState_e state;
    uint32_t* finish_tick;
    // 步骤
    enum Step_e {
      PREPARE,                // 准备
      TRAJ_RELAY_0,           // 移动至中间点0(x+)
      TRAJ_RELAY_1,           // 移动至中间点1(避免干涉)
      TRAJ_DEPOSIT_ABOVE,     // 移动至存矿点上方
      TRAJ_DEPOSIT_DOWNWARD,  // 下降至存矿点
      PUMP_0_ON,              // 开启存矿气泵
      PUMP_E_OFF,             // 关闭机械臂气泵
      TRAJ_DEPOSIT_UPWARD,    // 升高至存矿点上方
    } step;
    // 存矿位
    uint8_t side;
    // 存矿数记录(打开存矿气泵)
    uint32_t cnt[2];
    // 中间点0(正前方)
    Pose_t relay0 = Pose(0.235, 0, 0.3, 0, 0, 0);
    // 中间点1(左右，避免干涉)
    Pose_t relay1[2] = {Pose(0, 0.1, 0.4, PI * 0.25, PI * 0.25, 0),
                        Pose(0, -0.1, 0.4, PI * 0.25, PI * 0.25, 0)};
    // 存矿吸盘上方
    Pose_t deposit_above[2] = {Pose(-0.3, 0.2, 0.22, PI * 0.5, PI * 0.5, 0),
                               Pose(-0.3, -0.2, 0.22, PI * 0.5, PI * 0.5, 0)};
    // 存矿吸盘
    Pose_t deposit[2] = {Pose(-0.3, 0.2, 0.12, PI * 0.5, PI * 0.5, 0),
                         Pose(-0.3, -0.2, 0.12, PI * 0.5, PI * 0.5, 0)};

    // 处理函数
    void handle(void);
  } deposit_;

  // 出矿
  struct Withdraw_t {
    // 状态
    TaskState_e state;
    uint32_t* finish_tick;
    // 步骤
    enum Step_e {
      PREPARE,                // 准备
      TRAJ_RELAY_0,           // 移动至中间点0(x+)
      TRAJ_RELAY_1,           // 移动至中间点1(避免干涉)
      TRAJ_DEPOSIT_ABOVE,     // 移动至存矿点上方
      TRAJ_DEPOSIT_DOWNWARD,  // 下降至存矿点
      PUMP_E_ON,              // 开启机械臂气泵
      PUMP_0_OFF,             // 关闭存矿气泵
      TRAJ_DEPOSIT_UPWARD,    // 升高至存矿点上方
      TRAJ_RELAY_1_BACK,      // 移动至中间点1
      TRAJ_EXCHANGE,          // 移动至兑换默认位姿
    } step;
    // 出矿位
    uint8_t side;
    // 出矿数记录(打开存矿气泵)
    uint32_t cnt[2];
    // 中间点0(正前方)
    Pose_t relay0 = Pose(0.235, 0, 0.3, 0, 0, 0);
    // 中间点1(左右，避免干涉)
    Pose_t relay1[2] = {Pose(0, 0.1, 0.4, PI * 0.25, PI * 0.25, 0),
                        Pose(0, -0.1, 0.4, PI * 0.25, PI * 0.25, 0)};
    // 存矿吸盘上方
    Pose_t withdraw_above[2] = {Pose(-0.35, 0.2, 0.22, PI * 0.5, PI * 0.5, 0),
                                Pose(-0.35, -0.2, 0.22, PI * 0.5, PI * 0.5, 0)};
    // 存矿吸盘
    Pose_t withdraw[2] = {Pose(-0.35, 0.2, 0.12, PI * 0.5, PI * 0.5, 0),
                          Pose(-0.35, -0.2, 0.12, PI * 0.5, PI * 0.5, 0)};
    // 兑换默认位姿
    Pose_t exchange_default = Pose(0.2, 0, 0.4, 0, 0, 0);

    // 处理函数
    void handle(void);
  } withdraw_;

  // 兑换
  struct Exchange_t {
    // 状态
    TaskState_e state;
    uint32_t* finish_tick;
    // 步骤
    enum Step_e {
      PREPARE,       // 准备
      TRAJ_DEFAULT,  // 移动至默认位置
      LOCATE,        // 定位
      PUMP_E_OFF,    // 关闭机械臂气泵
      TRAJ_ROTATE,   // 末端旋转
    } step;
    // 兑换默认位姿
    Pose_t default_pose = Pose(0.2, 0, 0.4, 0, 0, 0);
    // 兑换开始位姿
    Pose_t start_pose;
    // 兑换旋转位姿
    Pose_t rotate_pose;
    float rotate_angle = math::deg2rad(45);

    // 处理函数
    void handle(void);
  } exchange_;

  // 三连取矿
  struct TriplePick_t {
    // 状态
    TaskState_e state;
    // 步骤
    enum Step_e {
      PREPARE,    // 准备
      PICK_1,     // 取1
      DEPOSIT_1,  // 存1
      PICK_2,     // 取2
      DEPOSIT_2,  // 存2
      PICK_3,     // 取3
    } step;
    // 矿石间隔
    float mine_dist = 0.27;
    // 3矿坐标
    Pose_t mine[3];

    // // 处理函数
    // void handle(void);
  } triple_pick_;
};

#endif  // CONTROL_H
