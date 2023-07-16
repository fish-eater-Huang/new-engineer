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
#include "app/arm_controller.h"
#include "app/chassis.h"
#include "app/gimbal.h"
#include "app/imu_comm.h"
#include "app/imu_monitor.h"
#include "app/mine.h"
#include "app/motor_monitor.h"
#include "app/pump.h"
#include "base/bsp/bsp_buzzer.h"
#include "base/bsp/bsp_led.h"
#include "base/common/math.h"
#include "base/cv_comm/cv_comm.h"
#include "base/referee_comm/referee_comm.h"
#include "base/remote/remote.h"
#include "base/servo/servo.h"

void iwdgHandler(bool iwdg_refresh_flag);
void robotPowerStateFSM(bool stop_flag);
void robotReset(void);
bool robotStartup(void);
void robotControl(void);
void boardLedHandle(void);

extern RC rc;
extern Arm arm;
extern ArmController arm_controller;
extern CVComm cv_comm;
extern RefereeComm referee;
// extern ServoZX361D pump_servo[];

MecanumChassis chassis(&CMFL, &CMFR, &CMBL, &CMBR, PID(5, 0, 10, 100, 240),
                       LowPassFilter(2e-2f));

ArmGimbal gimbal(&JM0, &GMP, &board_imu);

ServoPwm pump_e_servo(&htim8, TIM_CHANNEL_3);
ServoPwm pump_0_servo(&htim8, TIM_CHANNEL_4);
Pump pump_e(&PME, &pump_e_servo, nullptr, 1166, 500, 1833);  // 机械臂气泵
Pump pump_0(&PM0, &pump_0_servo, nullptr, 1166, 500, 1833);  // 存矿气泵

BoardLed led;

// 上电状态
enum RobotPowerState_e {
  STOP = 0,
  STARTUP = 1,
  WORKING = 2,
} robot_state;
// 初始化标志
bool startup_flag = false;
// 遥控器挡位记录
RC::RCSwitch last_rc_switch;

// 遥控器控制参数
namespace rcctrl {
const float arm_position_rate = 1e-6f;
const float arm_direction_rate = 3e-6f;
const float arm_joint_rate = 3e-6f;

const float chassis_speed_rate = 4e-3f;
const float chassis_rotate_rate = 0.5f;
const float chassis_follow_ff_rate = 0.3f;

const float gimbal_rate = 3e-4f;
}  // namespace rcctrl

// 控制初始化
void controlInit(void) {
  pump_0_servo.init();
  pump_e_servo.init();
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
    allMotorsOn();                  // 电机上电
    startup_flag = robotStartup();  // 开机状态判断
  } else if (robot_state == WORKING) {
    allMotorsOn();   // 电机上电
    robotControl();  // 机器人控制
  }

  chassis.rotateHandle(-gimbal.j0EncoderAngle());
  chassis.handle();
  JM0.control_data_.feedforward_intensity = JM1.intensity_float_;
  gimbal.handle();
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
    } else if (startup_flag) {
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
  startup_flag = false;
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
    // 机械臂x+y+yaw+pitch+roll
    arm.mode_ = Arm::Mode_e::MANIPULATION;
    arm.addRef(rc.channel_.l_col * rcctrl::arm_position_rate,
               -rc.channel_.l_row * rcctrl::arm_position_rate, 0,
               -rc.channel_.r_row * rcctrl::arm_direction_rate,
               -rc.channel_.r_col * rcctrl::arm_direction_rate,
               -rc.channel_.dial_wheel * rcctrl::arm_direction_rate);
    arm.trajAbort();  
//    if (rc.switch_.l != last_rc_switch.l || rc.switch_.r != last_rc_switch.r) {
//      arm.trajSet(0.235, 0, 0.15, 0, 0, 0, 0.6, 3);
//      arm.trajStart();
//    }

  }
  // 遥控器挡位左中右上
  else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::UP) {
    // 机械臂x+y+z+yaw+pitch
    arm.mode_ = Arm::Mode_e::MANIPULATION;
    arm.addRef(rc.channel_.l_col * rcctrl::arm_position_rate,
               -rc.channel_.l_row * rcctrl::arm_position_rate,
               -rc.channel_.dial_wheel * rcctrl::arm_position_rate,
               -rc.channel_.r_row * rcctrl::arm_direction_rate,
               -rc.channel_.r_col * rcctrl::arm_direction_rate, 0);
    arm.trajAbort();
//    if (rc.switch_.l != last_rc_switch.l || rc.switch_.r != last_rc_switch.r) {
//      arm.trajSet(0.235, 0, 0.4, 0, 0, 0, 0.6, 3);
//      arm.trajStart();
//    }
  }
  // 遥控器挡位左下右上
  else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::UP) {
    // 控制器档
    arm.mode_ = Arm::Mode_e::MANIPULATION;
    if (arm_controller.state_) {
      if (rc.switch_.l != last_rc_switch.l ||
          rc.switch_.r != last_rc_switch.r) {
        arm_controller.setOffset(arm.fdb_.x - arm_controller.raw_.x,
                                 arm.fdb_.y - arm_controller.raw_.y,
                                 arm.fdb_.z - arm_controller.raw_.z);
      } else {
        arm_controller.addOffset(
            rc.channel_.l_col * rcctrl::arm_position_rate,
            -rc.channel_.l_row * rcctrl::arm_position_rate,
            -rc.channel_.dial_wheel * rcctrl::arm_position_rate);
      }
      arm.setRef(arm_controller.ref_.x, arm_controller.ref_.y,
                 arm_controller.ref_.z, arm_controller.ref_.yaw,
                 arm_controller.ref_.pitch, arm_controller.ref_.roll);
    }
    arm.trajAbort();
//    if (rc.switch_.l != last_rc_switch.l || rc.switch_.r != last_rc_switch.r) {
//      arm.trajSet(-0.3, -0.1, 0.12, 0, 0, 0, 0.6, 3);
//      arm.trajStart();
//    }
  }
  // 遥控器挡位左上右中
  else if (rc.switch_.l == RC::UP && rc.switch_.r == RC::MID) {
    // 云台底盘测试
    arm.mode_ = Arm::Mode_e::JOINT;
    arm.trajAbort();

    chassis.lock_ = false;
//    chassis.mode_ = MecanumChassis::NORMAL;
//    chassis.setSpeed(rc.channel_.r_col * rcctrl::chassis_speed_rate,
//                     -rc.channel_.r_row * rcctrl::chassis_speed_rate,
//                     rc.channel_.dial_wheel * rcctrl::chassis_rotate_rate);
    chassis.mode_ = MecanumChassis::FOLLOW;
    if (fabs(rc.channel_.dial_wheel) > 100) {
      chassis.mode_ = MecanumChassis::GYRO;
      chassis.setSpeed(rc.channel_.r_col * rcctrl::chassis_speed_rate,
                       -rc.channel_.r_row * rcctrl::chassis_speed_rate,
                       rc.channel_.dial_wheel * rcctrl::chassis_rotate_rate);
    } else {
      chassis.setAngleSpeed(rc.channel_.r_col * rcctrl::chassis_speed_rate,
                            -rc.channel_.r_row * rcctrl::chassis_speed_rate, 0);
    }

    gimbal.addAngle(-rc.channel_.l_row * rcctrl::gimbal_rate,
                    -rc.channel_.l_col * rcctrl::gimbal_rate);
  }
  // 遥控器挡位左中右中
  else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::MID) {
    arm.mode_ = Arm::Mode_e::JOINT;
    arm.addJointRef(-rc.channel_.l_row * rcctrl::arm_joint_rate,
                    rc.channel_.l_col * rcctrl::arm_joint_rate,
                    rc.channel_.dial_wheel * rcctrl::arm_joint_rate,
                    rc.channel_.r_row * rcctrl::arm_joint_rate,
                    -rc.channel_.r_col * rcctrl::arm_joint_rate, 0);
    arm.trajAbort();
  }
  // 遥控器挡位左下右中
  else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::MID) {
    arm.mode_ = Arm::Mode_e::MANIPULATION;
    if (rc.switch_.l != last_rc_switch.l || rc.switch_.r != last_rc_switch.r) {
      arm.trajSet(0.15, 0, 0.15, 0, 0, 0, 0.5, 3);
      arm.trajStart();
    }

    // 存矿气泵控制
    if (pump_0.valve_state_ == Pump::ValveState_e::CLOSE) {
      if (rc.channel_.r_row == 660) {
        pump_0.set(pump_0.motor_speed_, Pump::OPEN_1);
      } else if (rc.channel_.r_row == -660) {
        pump_0.set(pump_0.motor_speed_, Pump::OPEN_2);
      }
    } else if (pump_0.valve_state_ == Pump::ValveState_e::OPEN_1) {
      if (rc.channel_.r_row < -300) {
        pump_0.set(pump_0.motor_speed_, Pump::CLOSE);
      }
    } else if (pump_0.valve_state_ == Pump::ValveState_e::OPEN_2) {
      if (rc.channel_.r_row > 300) {
        pump_0.set(pump_0.motor_speed_, Pump::CLOSE);
      }
    }
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
