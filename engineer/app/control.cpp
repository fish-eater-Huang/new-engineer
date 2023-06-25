/**
 ******************************************************************************
 * @file    control.cpp/h
 * @brief   Robot control design. 机器人控制设计（模式/键位）
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/control.h"

#include "iwdg.h"

#include "app/arm.h"
#include "app/board_comm.h"
#include "app/imu_monitor.h"
#include "app/motor_monitor.h"
#include "base/bsp/bsp_buzzer.h"
#include "base/bsp/bsp_led.h"
#include "base/common/math.h"
#include "base/remote/remote.h"

void iwdgHandler(bool iwdg_refresh_flag);
void robotPowerStateFSM(bool stop_flag);
void robotReset(void);
bool robotStartup(void);
void robotControl(void);
void boardLedHandle(void);

extern RC rc;
extern Arm arm;

uint8_t board_id = 0;
ArmController arm_controller(controller_imu);
BoardLed led;

// 上电状态
enum RobotPowerState_e {
  STOP = 0,
  STARTUP = 1,
  WORKING = 2,
} robot_state;
// 初始化标志
bool robot_init_flag = false;
// 遥控器挡位记录
RC::RCSwitch last_rc_switch;
// 额外功率
float extra_power_max = 0;

// 遥控器控制
namespace rcctrl {
const float arm_position_rate = 5e-7f;
const float arm_direction_rate = 3e-6f;
}  // namespace rcctrl

// 控制初始化
void controlInit(void) {
  led.init();
  robot_state = STOP;
}

// 控制主循环
void controlLoop(void) {
  iwdgHandler(rc.connect_.check());
  robotPowerStateFSM(!rc.connect_.check() || rc.switch_.r == RC::DOWN);

  if (robot_state == STOP) {
    allMotorsStopShutoff();
    robotReset();
  } else if (robot_state == STARTUP) {
    allMotorsOn();                     // 电机上电
    robot_init_flag = robotStartup();  // 开机状态判断
  } else if (robot_state == WORKING) {
    allMotorsOn();   // 电机上电
    robotControl();  // 机器人控制
  }

  arm_controller.handle();
  boardLedHandle();
}

// IWDG处理，true持续刷新，false进入STOP状态并停止刷新
void iwdgHandler(bool iwdg_refresh_flag) {
  if (!iwdg_refresh_flag) {
    robot_state = STOP;
  } else {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

// STOP(断电，安全模式)/开机/正常运行 状态机
void robotPowerStateFSM(bool stop_flag) {
  if (robot_state == STOP) {
    if (!stop_flag) {
      robot_state = STARTUP;
    }
  } else if (robot_state == STARTUP) {
    if (stop_flag) {
      robot_state = STOP;
    } else if (robot_init_flag) {
      // 初始化/复位完成
      robot_state = WORKING;
    }
  } else if (robot_state == WORKING) {
    if (stop_flag) {
      robot_state = STOP;
    }
  }
}

// 重置各功能状态
void robotReset(void) {
  robot_init_flag = false;
  arm.mode_ = Arm::Mode_e::STOP;
  last_rc_switch = rc.switch_;
}

// 开机上电启动处理
bool robotStartup(void) {
  bool flag = true;
  return flag;
}

// 机器人控制
void robotControl(void) {
  // 遥控器挡位左上右上
  if (rc.switch_.l == RC::UP && rc.switch_.r == RC::UP) {
    arm.mode_ = Arm::Mode_e::MANIPULATION;
    arm.addRef(rc.channel_.l_col * rcctrl::arm_position_rate,
               -rc.channel_.l_row * rcctrl::arm_position_rate,
               -rc.channel_.dial_wheel * rcctrl::arm_position_rate,
               -rc.channel_.r_row * rcctrl::arm_direction_rate,
               -rc.channel_.r_col * rcctrl::arm_direction_rate, 0);
  }
  // 遥控器挡位左中右上
  else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::UP) {
    arm.mode_ = Arm::Mode_e::MANIPULATION;
    arm.addRef(rc.channel_.l_col * rcctrl::arm_position_rate,
               -rc.channel_.l_row * rcctrl::arm_position_rate, 0,
               -rc.channel_.r_row * rcctrl::arm_direction_rate,
               -rc.channel_.r_col * rcctrl::arm_direction_rate,
               -rc.channel_.dial_wheel * rcctrl::arm_direction_rate);
  }
  // 遥控器挡位左下右上
  else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::UP) {
    arm.mode_ = Arm::Mode_e::MANIPULATION;
  }
  // 遥控器挡位左上右中
  else if (rc.switch_.l == RC::UP && rc.switch_.r == RC::MID) {
    arm.mode_ = Arm::Mode_e::COMPLIANCE;
  }
  // 遥控器挡位左中右中
  else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::MID) {
    arm.mode_ = Arm::Mode_e::COMPLIANCE;
  }
  // 遥控器挡位左下右中
  else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::MID) {
    if (rc.switch_.l != last_rc_switch.l || rc.switch_.r != last_rc_switch.r) {
      arm_controller.setOffset(arm.fdb_.x - arm_controller.raw_.x,
                               arm.fdb_.y - arm_controller.raw_.y,
                               arm.fdb_.z - arm_controller.raw_.z);
    }
    arm.mode_ = Arm::Mode_e::MANIPULATION;
    arm.setRef(arm_controller.ref_.x, arm_controller.ref_.y,
               arm_controller.ref_.z, arm_controller.ref_.yaw,
               arm_controller.ref_.pitch, arm_controller.ref_.roll);
  }

  // 记录遥控器挡位状态
  last_rc_switch = rc.switch_;
}

// 板载LED指示灯效果
void boardLedHandle(void) {
#ifdef DBC
  if (robot_state == STOP) {
    led.setColor(255, 0, 0);  // red
    led.setModeOn();
  } else if (robot_state == STARTUP) {
    led.setColor(150, 150, 0);  // yellow
    led.setModeBreath();
  } else if (robot_state == WORKING) {
    led.setColor(0, 0, 255);  // blue
    if (rc.switch_.l == RC::UP && rc.switch_.r == RC::UP) {
      // 遥控器挡位左上右上
      led.setModeBlink(1);
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::UP) {
      // 遥控器挡位左中右上
      led.setModeBlink(2);
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::UP) {
      // 遥控器挡位左下右上
      led.setModeBlink(3);
    } else if (rc.switch_.l == RC::UP && rc.switch_.r == RC::MID) {
      // 遥控器挡位左上右中
      led.setModeBlink(4);
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::MID) {
      // 遥控器挡位左中右中
      led.setModeBlink(5);
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::MID) {
      // 遥控器挡位左下右中
      led.setModeBlink(6);
    }
  }
  led.handle();
#elif defined DBA
  if (robot_state == STOP) {
    led.setLED(true, false, 0);  // red
  } else if (robot_state == STARTUP) {
    led.setLED(true, true, 0);  // red+green
  } else if (robot_state == WORKING) {
    if (rc.switch_.l == RC::UP && rc.switch_.r == RC::UP) {
      // 遥控器挡位左上右上
      led.setLED(false, true, 1);  // green
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::UP) {
      // 遥控器挡位左中右上
      led.setLED(false, true, 2);  // green
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::UP) {
      // 遥控器挡位左下右上
      led.setLED(false, true, 3);  // green
    } else if (rc.switch_.l == RC::UP && rc.switch_.r == RC::MID) {
      // 遥控器挡位左上右中
      led.setLED(false, true, 4);  // green
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::MID) {
      // 遥控器挡位左中右中
      led.setLED(false, true, 5);  // green
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::MID) {
      // 遥控器挡位左下右中
      led.setLED(false, true, 6);  // green
    }
  }
#endif
}

// 机械臂控制器构造函数
ArmController::ArmController(IMU imu[3]) {
  imu_[0] = &imu[0];
  imu_[1] = &imu[1];
  imu_[2] = &imu[2];
}

// 设置机械臂控制器位置偏置值
void ArmController::setOffset(float dx, float dy, float dz) {
  offset_.x = dx;
  offset_.y = dy;
  offset_.z = dz;
}

// 机械臂控制器处理函数
void ArmController::handle(void) {
  // 目标姿态
  raw_.yaw = math::deg2rad(imu_[2]->yaw());
  raw_.pitch = math::deg2rad(imu_[2]->pitch());
  raw_.roll = math::deg2rad(imu_[2]->roll());
  ref_.yaw = raw_.yaw;  // todo
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
