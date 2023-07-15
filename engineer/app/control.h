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
#include "base/remote/remote.h"

// 控制初始化
void controlInit(void);
// 控制主循环
void controlLoop(void);

// 机械臂任务类
class ArmTask {
  // 任务状态
  typedef enum State {
    DEFAULT,
    
  } State_e;

 public:
  ArmTask(/* args */);

 private:
  Arm* arm_;
  ArmController* controller_;
  RC* rc_;
};

#endif  // CONTROL_H