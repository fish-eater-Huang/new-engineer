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

#include "app/encoder.h"
#include "app/imu_comm.h"
#include "base/imu/imu.h"
#include "base/motor/motor.h"
#include "base/robotics/robotics.h"

// 机械臂类
class Arm {
 public:
  // 构造函数
  Arm(Motor* jm1, Motor* jm2, Motor* jm3, Motor* jm4, Motor* jm5, Motor* jm6,
      KKEncoder* encoder, IMU* imu0, IMU* imu2, IMU* imu3, ImuComm* imu_comm);

  // 初始化关节角度(非绝对式编码器, todo)
  void init(void);

  // 反馈状态解算，目标状态处理，运行控制器
  void handle(void);

  // 设置目标状态
  void setRef(const float& x, const float& y, const float& z, const float& yaw,
              const float& pitch, const float& roll);

  // 增量设置目标状态
  void addRef(const float& x, const float& y, const float& z, const float& yaw,
              const float& pitch, const float& roll);

  // 设置关节目标状态
  void setJointRef(const float& q1, const float& q2, const float& q3,
                   const float& q4, const float& q5, const float& q6);

  // 增量设置关节目标状态
  void addJointRef(const float& q1, const float& q2, const float& q3,
                   const float& q4, const float& q5, const float& q6);

  // 设置轨迹终点(末端位姿)+速度
  void trajSet(const float& x, const float& y, const float& z, const float& yaw,
               const float& pitch, const float& roll, const float& speed,
               const float& rotate_speed);

  // 设置轨迹终点(关节角度)+速度
  void trajSet(Matrixf<6, 1> q, Matrixf<6, 1> q_D1);

  // 开始轨迹
  uint32_t trajStart(void);

  // 中止轨迹
  void trajAbort(void);

 private:
  // 逆运动学求解(解析形式)
  Matrixf<6, 1> ikine(Matrixf<4, 4> T, Matrixf<6, 1> q0);

  // 停止状态控制器(电机断电)
  void stopController(void);

  // 操作空间控制器(末端位姿)
  void manipulationController(void);

  // 关节空间控制器(关节角度)
  void jointController(void);

  // 柔顺控制器
  void complianceController(void);

  // 轨迹规划器
  void trajectoryPlanner(void);

 public:
  // 机械臂模型
  robotics::Link links_[6] = {
      // link1
      robotics::Link(0, 0, 0, -PI / 2,           // theta,d,a,alpha
                     robotics::Joint_Type_e::R,  // joint type
                     0, 0, 0, 0.586,             // offset,qmin,qmax,m
                     Matrixf<3, 1>((float[3]){0, 0.056, 0.025}),  // rc
                     matrixf::zeros<3, 3>()),                     // I
      // link2
      robotics::Link(0, 0, 0.48, 0,              // theta,d,a,alpha
                     robotics::Joint_Type_e::R,  // joint type
                     0, 0, 0, 5.73,              // offset,qmin,qmax,m
                     Matrixf<3, 1>((float[3]){-0.248, 0, 0.071}),  // rc
                     matrixf::zeros<3, 3>()),                      // I
      // link3
      robotics::Link(0, 0, 0, -PI / 2,           // theta,d,a,alpha
                     robotics::Joint_Type_e::R,  // joint type
                     0, 0, 0, 1.334,             // offset,qmin,qmax,m
                     Matrixf<3, 1>((float[3]){0.002, 0.01, 0.101}),  // rc
                     matrixf::zeros<3, 3>()),                        // I
      // link4
      robotics::Link(0, 0.5125, 0, PI / 2,       // theta,d,a,alpha
                     robotics::Joint_Type_e::R,  // joint type
                     0, 0, 0, 1.447,             // offset,qmin,qmax,m
                     Matrixf<3, 1>((float[3]){-0.008, -0.128, 0}),  // rc
                     matrixf::zeros<3, 3>()),                       // I
      // link5
      robotics::Link(0, 0, 0, -PI / 2,           // theta,d,a,alpha
                     robotics::Joint_Type_e::R,  // joint type
                     0, 0, 0, 0.105,             // offset,qmin,qmax,m
                     Matrixf<3, 1>((float[3]){0, 0, 0.012}),  // rc
                     matrixf::zeros<3, 3>()),                 // I
      // link6
      robotics::Link(0, 0.025, 0, 0,                      // theta,d,a,alpha
                     robotics::Joint_Type_e::R,           // joint type
                     0, 0, 0, 0,                          // offset,qmin,qmax,m
                     Matrixf<3, 1>((float[3]){0, 0, 0}),  // rc
                     matrixf::zeros<3, 3>()),             // I
  };

  robotics::Serial_Link<6> arm_;

  // 电机指针
  Motor *jm1_, *jm2_, *jm3_, *jm4_, *jm5_, *jm6_;

  struct Init_t {
    // 初始化状态
    bool is_finish;
    // 初始化方式
    enum Method_e {
      ENCODER,   // 编码器
      LINK_IMU,  // IMU
      MANUAL,    // 手动
    } method[6];

    // 编码器指针
    KKEncoder* encoder;
    // 电机编码器零点
    const float encoder_zero[6] = {4.5, 0, 0, 40.8, -177.7, -135.6};

    // IMU指针
    IMU *imu0, *imu2, *imu3;
    ImuComm* imu_comm;
  } init_;

  // 工作模式
  enum Mode_e {
    STOP,
    MANIPULATION,
    JOINT,
    COMPLIANCE,
  } mode_;

  // 限位
  struct Limit_t {
    const float qmin[6] = {math::deg2rad(-160), math::deg2rad(-165),
                           math::deg2rad(-90),  math::deg2rad(-180),
                           math::deg2rad(-95),  math::deg2rad(-180)};
    const float qmax[6] = {math::deg2rad(160), math::deg2rad(0),
                           math::deg2rad(75),  math::deg2rad(180),
                           math::deg2rad(70),  math::deg2rad(180)};
    const float xmin = -0.6f;
    const float xmax = 0.66f;
    const float ymin = -0.6f;
    const float ymax = 0.6f;
    const float zmin = -0.6f;
    const float zmax = 0.57f;
  } limit_;

  // J6减速比
  const float ratio6_ = -27.f / 62.f / 2.f;

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
  Matrixf<3, 3> R0_;

  // 前馈力矩
  Matrixf<6, 1> torq_;

  // 轨迹规划
  struct Traj_t {
    // 轨迹规划运行状态
    bool state;

    // 轨迹规划方法(工作空间/关节空间插值)
    enum Method_e {
      MANIPULATION,
      JOINT,
    } method;

    // 轨迹起点
    struct Start_t {
      Matrixf<6, 1> q;         // rad
      float x, y, z;           // m
      float yaw, pitch, roll;  // rad
      Matrixf<3, 3> R;
      uint32_t tick;
    } start;

    // 轨迹终点
    struct End_t {
      Matrixf<6, 1> q;         // rad
      float x, y, z;           // m
      float yaw, pitch, roll;  // rad
      Matrixf<3, 3> R;
      uint32_t tick;
    } end;

    // 速度
    float speed;         // 速度(m/s)
    float rotate_speed;  // 角速度(rad/s)
    Matrixf<6, 1> q_D1;  // 关节角速度(rad/s)

    // 时间相关变量
    uint32_t ticks;
    float sigma;

    // 旋转向量
    Matrixf<4, 1> r_theta;
  } traj_;
};

#endif  // ARM_H
