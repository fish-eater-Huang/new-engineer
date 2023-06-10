/**
 ******************************************************************************
 * @file    arm.cpp/h
 * @brief   mechanical arm control. 机械臂控制
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/arm.h"

// 构造函数
Arm::Arm(Motor* j1, Motor* j2, Motor* j3, Motor* j4, Motor* j5, Motor* j6,
         Motor* j3_sup)
    : j1_(j1),
      j2_(j2),
      j3_(j3),
      j4_(j4),
      j5_(j5),
      j6_(j6),
      j3_sup_(j3_sup),
      arm_(links_) {}
