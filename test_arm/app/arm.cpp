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
#include "base/common/math.h"

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
      arm_(links_) {
  init();
}

// 初始化关节角度
void Arm::init(void) {
  // 初始角度设置
  float q_init_deg[6] = {0, 67.5f, -65.6f, -1.9f, 90.0f, 0};
  j1_->resetFeedbackAngle(q_init_deg[0]);
  j2_->resetFeedbackAngle(q_init_deg[1]);
  j3_->resetFeedbackAngle(q_init_deg[2]);
  j4_->resetFeedbackAngle(q_init_deg[3]);
  j5_->resetFeedbackAngle(q_init_deg[4]);
  j6_->resetFeedbackAngle(q_init_deg[5]);
  j3_sup_->resetFeedbackAngle(q_init_deg[2]);

  float q[6];
  for (int i = 0; i < 6; i++) {
    q[i] = math::deg2rad(q_init_deg[i]);
  }
  fdb_.q = Matrixf<6, 1>(q);
  fdb_.q_D1 = matrixf::zeros<6, 1>();

  fdb_.T = arm_.fkine(fdb_.q);
  R0 = robotics::t2r(arm_.fkine(matrixf::zeros<6, 1>()));
  Matrixf<3, 1> p_fdb = robotics::t2p(fdb_.T);
  fdb_.x = p_fdb[0][0];
  fdb_.y = p_fdb[1][0];
  fdb_.z = p_fdb[2][0];
  Matrixf<3, 1> rpy_fdb = robotics::r2rpy(robotics::t2r(fdb_.T) * R0.trans());
  fdb_.yaw = rpy_fdb[0][0];
  fdb_.pitch = rpy_fdb[1][0];
  fdb_.roll = rpy_fdb[2][0];

  ref_.q = fdb_.q;
  ref_.T = fdb_.T;
  ref_.x = fdb_.x;
  ref_.y = fdb_.y;
  ref_.z = fdb_.z;
  ref_.yaw = fdb_.yaw;
  ref_.pitch = fdb_.pitch;
  ref_.roll = fdb_.roll;
}

// 设置目标状态
void Arm::setRef(float x, float y, float z, float yaw, float pitch,
                 float roll) {
  ref_.x = x;
  ref_.y = y;
  ref_.z = z;
  ref_.yaw = yaw;
  ref_.pitch = pitch;
  ref_.roll = roll;
}

// 增量设置目标状态
void Arm::addRef(float x, float y, float z, float yaw, float pitch,
                 float roll) {
  ref_.x += x;
  ref_.y += y;
  ref_.z += z;
  ref_.yaw += yaw;
  ref_.pitch += pitch;
  ref_.roll += roll;
}

// 反馈状态解算，目标状态处理，运行控制器
void Arm::handle(void) {
  // 获取关节角度&角速度
  fdb_.q[0][0] = math::deg2rad(j1_->realAngle());
  fdb_.q[1][0] = math::deg2rad(j2_->realAngle());
  fdb_.q[2][0] = math::deg2rad(j3_->realAngle());
  fdb_.q[3][0] = math::deg2rad(j4_->realAngle());
  fdb_.q[4][0] = math::deg2rad(j5_->realAngle());
  fdb_.q[5][0] = math::deg2rad(j6_->realAngle());
  fdb_.q_D1[0][0] = math::dps2radps(j1_->realSpeed());
  fdb_.q_D1[1][0] = math::dps2radps(j2_->realSpeed());
  fdb_.q_D1[2][0] = math::dps2radps(j3_->realSpeed());
  fdb_.q_D1[3][0] = math::dps2radps(j4_->realSpeed());
  fdb_.q_D1[4][0] = math::dps2radps(j5_->realSpeed());
  fdb_.q_D1[5][0] = math::dps2radps(j6_->realSpeed());

  // 反馈位姿解算(正运动学)
  fdb_.T = arm_.fkine(fdb_.q);
  fdb_.J = arm_.jacob(fdb_.q);
  Matrixf<3, 1> p_fdb = robotics::t2p(fdb_.T);
  fdb_.x = p_fdb[0][0];
  fdb_.y = p_fdb[1][0];
  fdb_.z = p_fdb[2][0];
  Matrixf<3, 1> rpy_fdb = robotics::r2rpy(robotics::t2r(fdb_.T) * R0.trans());
  fdb_.yaw = rpy_fdb[0][0];
  fdb_.pitch = rpy_fdb[1][0];
  fdb_.roll = rpy_fdb[2][0];

  // 控制器
  if (mode_ == Arm::Mode_e::MANIPULATION) {
    manipulationController();
  } else if (mode_ == Arm::Mode_e::JOINT) {
    jointController();
  } else if (mode_ == Arm::Mode_e::STOP) {
    stopController();
  }

  // 电机控制
  j1_->setAngleSpeed(math::rad2deg(ref_.q[0][0]), 0,
                     j1_->model_(torq_[0][0], 0));
  j2_->setAngleSpeed(math::rad2deg(ref_.q[1][0]), 0,
                     j2_->model_(torq_[1][0], 0));
  j3_->setAngleSpeed(math::rad2deg(ref_.q[2][0]), 0,
                     j3_->model_(torq_[2][0], 0));
  j3_sup_->setAngleSpeed(math::rad2deg(ref_.q[2][0]), 0,
                         j3_sup_->model_(torq_[2][0], 0));
  j4_->setAngleSpeed(math::rad2deg(ref_.q[3][0]), 0,
                     j4_->model_(torq_[3][0], 0));
  j5_->setAngleSpeed(math::rad2deg(ref_.q[4][0]), 0,
                     j5_->model_(torq_[4][0], 0));
  j6_->setAngleSpeed(math::rad2deg(ref_.q[5][0]), 0,
                     j6_->model_(torq_[5][0], 0));
}

// 逆运动学求解(解析形式)
Matrixf<6, 1> Arm::ikine(Matrixf<4, 4> T) {
  Matrixf<6, 1> q;
  // (1)
  Matrixf<3, 3> R06 = robotics::t2r(T);
  Matrixf<3, 1> o6 = robotics::t2p(T);
  Matrixf<3, 1> z6 = T.block<3, 1>(0, 2);
  // (2)
  Matrixf<3, 1> z5 = z6;
  Matrixf<3, 1> o5 = o6 - links_[5].dh_.d * z5;
  // 处理奇异输入
  float d234 = links_[1].dh_.d + links_[2].dh_.d + links_[3].dh_.d;
  float a2 = links_[1].dh_.a;
  float a3 = links_[2].dh_.a;
  if (o5.norm() > (a2 + a3) * 0.85) {
    o5 = o5 / o5.norm() * (a2 + a3) * 0.85;
  } else if (sqrtf(o5[0][0] * o5[0][0] + o5[1][0] * o5[1][0]) <
             fabs(d234) * 2.0f) {
    o5[0][0] = o5[0][0] / sqrtf(o5[0][0] * o5[0][0] + o5[1][0] * o5[1][0]) *
               fabs(d234) * 2.0f;
    o5[1][0] = o5[1][0] / sqrtf(o5[0][0] * o5[0][0] + o5[1][0] * o5[1][0]) *
               fabs(d234) * 2.0f;
  }
  // (3)
  float xo5 = o5[0][0];
  float yo5 = o5[1][0];
  if (fabs(d234) < 1e-8f) {
    q[0][0] = atan2f(yo5, xo5);
  } else {
    q[0][0] = atan2f(yo5, xo5) +
              acosf(fabs(d234) / sqrtf(xo5 * xo5 + yo5 * yo5)) +
              fabs(d234) / d234 * PI / 2;
  }
  // (4)
  float r01[9] = {cosf(q[0][0]),
                  0,
                  sinf(q[0][0]),
                  sinf(q[0][0]),
                  0,
                  -cos(q[0][0]),
                  0,
                  1,
                  0};
  Matrixf<3, 3> R01(r01);
  Matrixf<3, 1> z1 = R01.block<3, 1>(0, 2);
  Matrixf<3, 1> z4_raw = vector3f::cross(z1, z5);
  static Matrixf<3, 1> z4;
  if (z4_raw.norm() > 1e-8f) {
    z4 = z4_raw / z4_raw.norm();
  }
  Matrixf<3, 1> o4 = o5 - links_[4].dh_.d * z4;
  // (5)
  Matrixf<3, 1> o1 = matrixf::zeros<3, 1>();
  o1[2][0] = links_[0].dh_.d;
  Matrixf<3, 1> p14 = o4 - o1;
  float phi =
      acosf((a2 * a2 + a3 * a3 + d234 * d234 - p14.norm() * p14.norm()) /
            (2 * a2 * a3));
  q[2][0] = phi - PI / 2;
  // (6)
  Matrixf<3, 1> y1 = matrixf::zeros<3, 1>();
  y1[2][0] = 1;
  Matrixf<3, 1> x1 = vector3f::cross(y1, z1);
  float gamma =
      asinf(a3 * sinf(phi) / sqrtf(p14.norm() * p14.norm() - d234 * d234));
  q[1][0] = atan2f((y1.trans() * p14)[0][0], (x1.trans() * p14)[0][0]) + gamma -
            PI / 2;
  // (7)
  Matrixf<3, 1> y4 = matrixf::zeros<3, 1>() - z1;
  Matrixf<3, 1> x4 = vector3f::cross(y4, z4);
  float r04[9] = {x4[0][0], y4[0][0], z4[0][0], x4[1][0], y4[1][0],
                  z4[1][0], x4[2][0], y4[2][0], z4[2][0]};
  Matrixf<3, 3> R04 = Matrixf<3, 3>(r04);
  float r13[9] = {cosf(q[1][0] + q[2][0]),
                  -sinf(q[1][0] + q[2][0]),
                  0,
                  sinf(q[1][0] + q[2][0]),
                  cosf(q[1][0] + q[2][0]),
                  0,
                  0,
                  0,
                  1};
  Matrixf<3, 3> R13 = Matrixf<3, 3>(r13);
  Matrixf<3, 3> R34 = (R01 * R13).trans() * R04;
  q[3][0] = atan2f(R34[1][0], R34[0][0]);
  // 校验q4
  if (fabs(math::loopLimit(q[3][0], -PI, PI)) > PI / 2) {
    z4 *= -1;
    // goto (4)
    o4 = o5 - links_[4].dh_.d * z4;
    // (5)
    o1 = matrixf::zeros<3, 1>();
    o1[2][0] = links_[0].dh_.d;
    p14 = o4 - o1;
    phi = acosf((a2 * a2 + a3 * a3 + d234 * d234 - p14.norm() * p14.norm()) /
                (2 * a2 * a3));
    q[2][0] = phi - PI / 2;
    // (6)
    y1 = matrixf::zeros<3, 1>();
    y1[2][0] = 1;
    x1 = vector3f::cross(y1, z1);
    gamma =
        asinf(a3 * sinf(phi) / sqrtf(p14.norm() * p14.norm() - d234 * d234));
    q[1][0] = atan2f((y1.trans() * p14)[0][0], (x1.trans() * p14)[0][0]) +
              gamma - PI / 2;
    // (7)
    y4 = matrixf::zeros<3, 1>() - z1;
    x4 = vector3f::cross(y4, z4);
    float r04[9] = {x4[0][0], y4[0][0], z4[0][0], x4[1][0], y4[1][0],
                    z4[1][0], x4[2][0], y4[2][0], z4[2][0]};
    R04 = Matrixf<3, 3>(r04);
    float r13[9] = {cosf(q[1][0] + q[2][0]),
                    -sinf(q[1][0] + q[2][0]),
                    0,
                    sinf(q[1][0] + q[2][0]),
                    cosf(q[1][0] + q[2][0]),
                    0,
                    0,
                    0,
                    1};
    R13 = Matrixf<3, 3>(r13);
    R34 = (R01 * R13).trans() * R04;
    q[3][0] = atan2f(R34[1][0], R34[0][0]);
  }
  // (8)
  Matrixf<3, 1> y5 = z4;
  Matrixf<3, 1> x5 = vector3f::cross(y5, z5);
  float r05[9] = {x5[0][0], y5[0][0], z5[0][0], x5[1][0], y5[1][0],
                  z5[1][0], x5[2][0], y5[2][0], z5[2][0]};
  Matrixf<3, 3> R05 = Matrixf<3, 3>(r05);
  Matrixf<3, 3> R45 = R04.trans() * R05;
  q[4][0] = atan2f(R45[1][0], R45[0][0]) - PI / 2;
  // (9)
  Matrixf<3, 3> R56 = R05.trans() * R06;
  q[5][0] = atan2f(R56[1][0], R56[0][0]);

  return q;
}

// 操作空间控制器(末端位姿)
void Arm::manipulationController(void) {
  // 目标状态限制
  //  ref_.x = math::limit(ref_.x, 0.2f, 0.8f);
  //  ref_.y = math::limit(ref_.y, -0.3f, 0.3f);
  //  ref_.z = math::limit(ref_.z, 0.5f, 1.0f);
  //  ref_.yaw = math::limit(ref_.yaw, -PI / 2, PI / 2);
  //  ref_.pitch = math::limit(ref_.pitch, -60.0f, 0);
  //  ref_.roll = math::limit(ref_.roll, -45.0f, 45.0f);

  // 目标状态解算
  Matrixf<3, 1> p_ref;
  p_ref[0][0] = ref_.x;
  p_ref[1][0] = ref_.y;
  p_ref[2][0] = ref_.z;
  Matrixf<3, 1> rpy_ref;
  rpy_ref[0][0] = ref_.yaw;
  rpy_ref[1][0] = ref_.pitch;
  rpy_ref[2][0] = ref_.roll;
  ref_.T = robotics::rp2t(robotics::rpy2r(rpy_ref) * R0, p_ref);

  // 逆运动学解析解
  ref_.q = ikine(ref_.T);

  // 动力学前馈(仅取位置项)
  torq_ = arm_.rne(fdb_.q);
}

// 关节空间控制器(关节角度)
void Arm::jointController(void) {
  // todo
}

// 停止状态控制器(电机断电/阻尼模式)
void Arm::stopController(void) {
  // 重置目标状态
  ref_.q = fdb_.q;
  ref_.T = fdb_.T;
  ref_.x = fdb_.x;
  ref_.y = fdb_.y;
  ref_.z = fdb_.z;
  ref_.yaw = fdb_.yaw;
  ref_.pitch = fdb_.pitch;
  ref_.roll = fdb_.roll;

  // 力矩前馈清零
  torq_ = matrixf::zeros<6, 1>();
}
