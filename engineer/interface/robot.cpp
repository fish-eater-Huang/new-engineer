/**
 ******************************************************************************
 * @file    robot.cpp/h
 * @brief   Main program. 主程序
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "interface/robot.h"

#include "cmsis_os.h"
#include "hardware_config.h"

#include "app/can_monitor.h"
#include "app/control.h"
#include "app/imu_monitor.h"
#include "app/motor_monitor.h"
#include "app/serial_tool.h"
#include "base/cap_comm/cap_comm.h"
#include "base/cv_comm/cv_comm.h"
#include "base/referee_comm/referee_comm.h"
#include "base/remote/remote.h"
#include "base/robotics/robotics.h"
#include "base/servo/servo.h"

#ifdef RC_UART
RC rc(RC_UART);
#else
RC rc;
#endif  // RC_UART
#ifdef CV_UART
CVComm cv_comm(CV_UART);
#else
CVComm cv_comm;
#endif  // CV_UART
#ifdef REFEREE_UART
RefereeComm referee(REFEREE_UART);
#else
RefereeComm referee;
#endif  // REFEREE_UART
#ifdef SERVO_UART
ServoZX361D gate_servo(SERVO_UART);
#else
ServoZX361D gate_servo;
#endif  // REFEREE_UART
#ifdef DEBUG_UART
SerialStudio serial_tool(DEBUG_UART);
#else
SerialStudio serial_tool;
#endif  // DEBUG_UART

CapComm ultra_cap(&hcan2);

/* FreeRTOS tasks-----------------------------------------------------------*/
osThreadId controlTaskHandle;
void controlTask(void const* argument) {
  rc.init();
  controlInit();
  for (;;) {
    rc.handle();
    controlLoop();
    osDelay(1);
  }
}

osThreadId motorTaskHandle;
void motorTask(void const* argument) {
  allMotorsInit();
  for (;;) {
    allMotorsHandle();
    osDelay(1);
  }
}

osThreadId canTaskHandle;
void canTask(void const* argument) {
  canFilterInit();
  for (;;) {
    canTxMonitor();
  }
}

osThreadId imuTaskHandle;
void imuTask(void const* argument) {
  imu::initAll();
  for (;;) {
    imu::handleAll();
    osDelay(1);
  }
}

// robotics test
float m[6] = {0.2645, 0.17, 0.1705, 0, 0, 0};
Matrixf<3, 6> rc_((float[18]){0, -8.5e-2, 0, 0, 0, 0, 13.225e-2, 0, 0, 0, 0, 0,
                              0, 3.7e-2, 8.525e-2, 0, 0, 0});
Matrixf<3, 3> I[6] = {
    matrixf::diag<3, 3>(Matrixf<3, 1>((float[3]){1.542e-3, 0, 1.542e-3})),
    matrixf::diag<3, 3>(Matrixf<3, 1>((float[3]){0, 0.409e-3, 0.409e-3})),
    matrixf::diag<3, 3>(Matrixf<3, 1>((float[3]){0.413e-3, 0.413e-3, 0})),
    3.0f * matrixf::eye<3, 3>(),
    2.0f * matrixf::eye<3, 3>(),
    1.0f * matrixf::eye<3, 3>(),
};
robotics::Link links[6]{
    robotics::Link(0, 26.45e-2, 0, -PI / 2, robotics::Joint_Type_e::R, 0, 0, 0,
                   m[0], rc_.col(0), I[0]),
    robotics::Link(0, 5.5e-2, 17e-2, 0, robotics::Joint_Type_e::R, 0, 0, 0,
                   m[1], rc_.col(1), I[1]),
    robotics::Link(0, 0, 0, -PI / 2, robotics::Joint_Type_e::R, 0, 0, 0, m[2],
                   rc_.col(2), I[2]),
    robotics::Link(0, 17.05e-2, 0, PI / 2, robotics::Joint_Type_e::R, 0, 0, 0,
                   m[3], rc_.col(3), I[3]),
    robotics::Link(0, 0, 0, -PI / 2, robotics::Joint_Type_e::R, 0, 0, 0, m[4],
                   rc_.col(4), I[4]),
    robotics::Link(0, 0, 0, 0, robotics::Joint_Type_e::R, 0, 0, 0, m[5],
                   rc_.col(5), I[5]),
};
robotics::Serial_Link<6> p560(links);

Matrixf<4, 4> T_test;
Matrixf<6, 6> J_test;
Matrixf<6, 1> torq_test;

osThreadId roboticsTaskHandle;
void roboticsTask(void const* argument) {
  for (;;) {
    float q[6] = {0.2, -0.5, -0.3, -0.6, 0.5, 0.2};
    float qv[6] = {1, 0.5, -1, 0.3, 0, -1};
    float qa[6] = {0.2, -0.3, 0.1, 0, -1, 0};
    float he[6] = {1, 2, -3, -0.5, -2, 1};
    T_test = p560.fkine(q);
    J_test = p560.jacob(q);
    torq_test = p560.rne(q, qv, qa, he);
    osDelay(1);
  }
}

osThreadId minipcCommTaskHandle;
void minipcCommTask(void const* argument) {
  uint32_t tick = osKernelSysTick();
  cv_comm.init();
  for (;;) {
    cv_comm.txMonitor(&tick);
  }
}

osThreadId refereeCommTaskHandle;
void refereeCommTask(void const* argument) {
  uint32_t tick = osKernelSysTick();
  referee.init();
  for (;;) {
    referee.handle();
    osDelayUntil(&tick, 10);
  }
}

osThreadId serialToolTaskHandle;
void serialToolTask(void const* argument) {
  uint32_t tick = osKernelSysTick();
  for (;;) {
    serial_tool.handle();
    osDelayUntil(&tick, 20);
  }
}

// Create and config tasks
void rtosTaskInit(void) {
  osThreadDef(control_task, controlTask, osPriorityAboveNormal, 0, 256);
  controlTaskHandle = osThreadCreate(osThread(control_task), NULL);

  osThreadDef(motor_task, motorTask, osPriorityHigh, 0, 128);
  motorTaskHandle = osThreadCreate(osThread(motor_task), NULL);

  osThreadDef(can_task, canTask, osPriorityHigh, 0, 128);
  canTaskHandle = osThreadCreate(osThread(can_task), NULL);

  osThreadDef(imu_task, imuTask, osPriorityRealtime, 0, 512);
  imuTaskHandle = osThreadCreate(osThread(imu_task), NULL);

  osThreadDef(minipc_comm_task, minipcCommTask, osPriorityNormal, 0, 512);
  minipcCommTaskHandle = osThreadCreate(osThread(minipc_comm_task), NULL);

  osThreadDef(referee_comm_task, refereeCommTask, osPriorityNormal, 0, 512);
  refereeCommTaskHandle = osThreadCreate(osThread(referee_comm_task), NULL);

  osThreadDef(robotics_task, roboticsTask, osPriorityNormal, 0, 1800);
  roboticsTaskHandle = osThreadCreate(osThread(robotics_task), NULL);

  osThreadDef(serial_tool_task, serialToolTask, osPriorityLow, 0, 1024);
  serialToolTaskHandle = osThreadCreate(osThread(serial_tool_task), NULL);
}
