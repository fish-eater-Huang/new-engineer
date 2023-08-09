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
    MOVE_HIGH,    // 行驶(机械臂伸出)
    PICK_NORMAL,  // 取矿(资源岛)
    PICK_LOW,     // 取矿(地面)/障碍块/救援
    PICK_HIGH,    // 取矿(空接)
    DEPOSIT_0,    // 存矿0(左)
    DEPOSIT_1,    // 存矿1(右)
    WITHDRAW_0,   // 出矿0(左)
    WITHDRAW_1,   // 出矿1(右)
    WITHDRAW_U0,  // 出矿0(左)
    WITHDRAW_U1,  // 出矿1(右)
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

  // 中止任务
  void abort(void);

  // 任务处理
  void handle(void);

  // 开始取矿
  void startPick(PickMethod_e method);

  // 开始兑换
  void startExchange(void);

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
    // 类型
    uint8_t type;
    // 机械臂中间点关节角度
    float q_relay[6] = {0, -2.35, 1.0, 0, -0.22, 0};
    // 机械臂收回关节角度
    float q_retract[6] = {0, -2.87, 1.3, 0, 0, 0};

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
    float default_pose[3][6] = {
        {0, -1.81, 0.59, 0, 1.2, 0},     // normal
        {0, -1.22, -0.53, 0, -1.60, 0},  // high
        {0, -0.75, 0.56, 0, 0.3, 0},     // low
    };
    // 定位位姿
    Pose_t start_pose;
    // 取矿升高距离
    Pose_t pick_up_offset[3] = {
        Pose_t(-0.15, 0, 0.3, 0, -0.4, 0),
        Pose_t(0, 0, 0, 0, 0, 0),
        Pose_t(0, 0, 0.2, 0, 0, 0),
    };
    Pose_t pick_up_pose;

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
      TRAJ_RELAY,             // 移动至中间点(避免干涉)
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
    // 中间点(左右，避免干涉)
    float relay[2][6] = {
        {1.5, -2.0, 0.2, 0, 1.2, -0.7},  // left
        {-1.5, -2.0, 0.2, 0, 1.2, 0.7},  // right
    };
    // 存矿吸盘上方
    float deposit_above[2][6] = {
        {2.65, -1.73, 0.54, 0, 1.2, -0.7},  // left
        {-2.65, -1.73, 0.54, 0, 1.2, 0.7},  // right
    };
    // 存矿吸盘
    float deposit[2][6] = {
        {2.65, -1.56, 0.68, 0, 0.89, -0.7},  // left
        {-2.65, -1.56, 0.68, 0, 0.89, 0.7},  // right
    };

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
      TRAJ_DEPOSIT_FRONT,     // 移动至存矿点上方
      TRAJ_DEPOSIT_BACKWARD,  // 下降至存矿点
      PUMP_E_ON,              // 开启机械臂气泵
      PUMP_0_OFF,             // 关闭存矿气泵
      TRAJ_DEPOSIT_UPWARD,    // 升高至存矿点上方
      TRAJ_EXCHANGE,          // 移动至兑换默认位姿
    } step;
    // 出矿位
    uint8_t side;
    // 存矿吸盘前方
    float withdraw_front[2][6] = {
        {1.51, -1.55, 1.10, -1.42, -1.41, 0.48},   // left
        {-1.51, -1.55, 1.10, 1.42, -1.41, -0.48},  // right
    };
    // 存矿吸盘
    float withdraw[2][6] = {
        {2.41, -1.44, 0.97, -0.91, -1.19, 0.40},   // left
        {-2.41, -1.44, 0.97, 0.91, -1.19, -0.40},  // right
    };
    // 存矿吸盘上方
    float withdraw_above[2][6] = {
        {2.40, -1.90, 0.84, -1.14, -0.95, 0.92},   // left
        {-2.40, -1.90, 0.84, 1.14, -0.95, -0.92},  // right
    };
    // 兑换默认位姿
    // float exchange_default[6] = {0, -2.3, 0.5, 0, 0.2, 0};
    float exchange_default[6] = {0, -2.35, 1.0, 0, -0.22, 0};

    // 处理函数
    void handle(void);
  } withdraw_;

  // 出矿(上表面)
  struct WithdrawAbove_t {
    // 状态
    TaskState_e state;
    uint32_t* finish_tick;
    // 步骤
    enum Step_e {
      PREPARE,                // 准备
      TRAJ_RELAY,             // 移动至中间点(避免干涉)
      TRAJ_DEPOSIT_ABOVE,     // 移动至存矿点上方
      TRAJ_DEPOSIT_DOWNWARD,  // 下降至存矿点
      PUMP_E_ON,              // 开启机械臂气泵
      PUMP_0_OFF,             // 关闭存矿气泵
      TRAJ_DEPOSIT_UPWARD,    // 升高至存矿点上方
      TRAJ_EXCHANGE,          // 移动至兑换默认位姿
    } step;
    // 出矿位
    uint8_t side;
    // 中间点(左右，避免干涉)
    float relay[2][6] = {
        {1.5, -2.0, 0.2, 0, 1.2, -0.7},  // left
        {-1.5, -2.0, 0.2, 0, 1.2, 0.7},  // right
    };
    // 存矿吸盘上方
    float withdraw_above[2][6] = {
        {2.65, -1.73, 0.54, 0, 1.2, -0.7},  // left
        {-2.65, -1.73, 0.54, 0, 1.2, 0.7},  // right
    };
    // 存矿吸盘
    float withdraw[2][6] = {
        {2.65, -1.56, 0.68, 0, 0.89, -0.7},  // left
        {-2.65, -1.56, 0.68, 0, 0.89, 0.7},  // right
    };
    // 兑换默认位姿
    // float exchange_default[6] = {0, -2.3, 0.5, 0, 0.2, 0};
    float exchange_default[6] = {0, -2.35, 1.0, 0, -0.22, 0};

    // 处理函数
    void handle(void);
  } withdraw_above_;

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
      TRAJ_PUSH_IN,  // 推入
    } step;
    // 兑换默认位姿
    float default_pose[6] = {0, -2.3, 0.5, 0, 0.2, 0};
    // 推入量
    Matrixf<3, 1> push_in_offset = Matrixf<3, 1>((float[3]){0, 0, 0.2});
    // 兑换开始位姿
    Pose_t start_pose;
    Pose_t push_in_pose;

    // 处理函数
    void handle(void);
  } exchange_;

  // 三连取矿
  struct TriplePick_t {
    // 状态
    TaskState_e state;
    // 步骤
    enum Step_e {
      PREPARE,       // 准备
      PICK_1,        // 取1
      DEPOSIT_1,     // 存1
      TRAJ_RELAY12,  // 12中间点
      PICK_2_ABOVE,  // 取2上方
      PICK_2,        // 取2
      DEPOSIT_2,     // 存2
      TRAJ_RELAY23,  // 12中间点
      PICK_3_ABOVE,  // 取3上方
      PICK_3,        // 取3
    } step;
    // 矿石间隔
    Matrixf<3, 1> mine_offset[3] = {
        Matrixf<3, 1>((float[3]){0, 0, 0}),
        Matrixf<3, 1>((float[3]){0, -0.3, 0}),
        Matrixf<3, 1>((float[3]){0, 0.3, 0}),
    };
    // 3矿坐标
    Pose_t mine[3];
    // 矿上方坐标
    Pose_t mine_above[3];
    // 中间点坐标
    float relay12[6] = {-1.5, -2.0, 0.2, 0, 1.2, 0.7};
    float relay23[6] = {1.0, -2.0, 0.2, 0, 1.2, -0.7};

    // // 处理函数
    // void handle(void);
  } triple_pick_;
};

#endif  // CONTROL_H
