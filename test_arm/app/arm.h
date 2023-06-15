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

#ifndef ARM_H
#define ARM_H

#include "app/motor_monitor.h"
#include "base/robotics/robotics.h"

// 机械臂类
class Arm {
 public:
  // 构造函数
  Arm(Motor* j1, Motor* j2, Motor* j3, Motor* j4, Motor* j5, Motor* j6,
      Motor* j3_sup);

  // 初始化关节角度(非绝对式编码器, todo)
  void init(void);

  // 设置目标状态
  void setRef(float x, float y, float z, float yaw, float pitch, float roll);

  // 增量设置目标状态
  void addRef(float x, float y, float z, float yaw, float pitch, float roll);

  // 反馈状态解算，目标状态处理，运行控制器
  void handle(void);

 private:
  // 逆运动学求解(解析形式)
  Matrixf<6, 1> ikine(Matrixf<4, 4> T);

  // 操作空间控制器(末端位姿)
  void manipulationController(void);

  // 关节空间控制器(关节角度)
  void jointController(void);

  // 停止状态控制器(电机断电/阻尼模式)
  void stopController(void);

 public:
  // 机械臂模型
  robotics::Link links_[6] = {
      // link1
      robotics::Link(0, 0, 0, -PI / 2,           // theta,d,a,alpha
                     robotics::Joint_Type_e::R,  // joint type
                     0, 0, 0, 0.83,              // offset,qmin,qmax,m
                     Matrixf<3, 1>((float[3]){0, 0.003, 0.049}),  // rc
                     matrixf::zeros<3, 3>()),                     // I

      // link2
      robotics::Link(0, 0, 0.266, 0,             // theta,d,a,alpha
                     robotics::Joint_Type_e::R,  // joint type
                     0, 0, 0, 1.222,             // offset,qmin,qmax,m
                     Matrixf<3, 1>((float[3]){-0.06, 0, 0.062}),  // rc
                     matrixf::zeros<3, 3>()),                     // I

      // link3
      robotics::Link(0, 0, 0, -PI / 2,           // theta,d,a,alpha
                     robotics::Joint_Type_e::R,  // joint type
                     0, 0, 0, 0.538,             // offset,qmin,qmax,m
                     Matrixf<3, 1>((float[3]){-0.004, 0, 0.012}),  // rc
                     matrixf::zeros<3, 3>()),                      // I

      // link4
      robotics::Link(0, 0.28, 0, PI / 2,         // theta,d,a,alpha
                     robotics::Joint_Type_e::R,  // joint type
                     0, 0, 0, 0.342,             // offset,qmin,qmax,m
                     Matrixf<3, 1>((float[3]){0, -0.327, 0.035}),  // rc
                     matrixf::zeros<3, 3>()),                      // I

      // link5
      robotics::Link(0, 0, 0, -PI / 2,           // theta,d,a,alpha
                     robotics::Joint_Type_e::R,  // joint type
                     0, 0, 0, 0.162,             // offset,qmin,qmax,m
                     Matrixf<3, 1>((float[3]){0, -0.02, 0}),  // rc
                     matrixf::zeros<3, 3>()),                 // I

      // link6
      robotics::Link(0, 0.07, 0, 0,              // theta,d,a,alpha
                     robotics::Joint_Type_e::R,  // joint type
                     0, 0, 0, 0.12,              // offset,qmin,qmax,m
                     Matrixf<3, 1>((float[3]){0, 0, -0.015}),  // rc
                     matrixf::zeros<3, 3>()),                  // I
  };

  robotics::Serial_Link<6> arm_;

  // 电机指针
  Motor *j1_, *j2_, *j3_, *j4_, *j5_, *j6_, *j3_sup_;

  // 工作模式
  enum Mode_e {
    STOP,
    MANIPULATION,
    JOINT,
  } mode_;

  // 目标状态
  struct Ref_t {
    Matrixf<6, 1> q;  // rad
    Matrixf<4, 4> T;

    float x, y, z;           // m
    float yaw, pitch, roll;  // rad
  } ref_;

  // 反馈状态
  struct Fdb_t {
    Matrixf<6, 1> q, q_D1;  // rad, rad/s
    Matrixf<4, 4> T;
    Matrixf<6, 6> J;

    float x, y, z;           // m
    float yaw, pitch, roll;  // rad
  } fdb_;

  // 默认旋转矩阵
  Matrixf<3, 3> R0;

  // 前馈力矩
  Matrixf<6, 1> torq_;
};

#endif  // ARM_H