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

#include "app/board_comm.h"
#include "app/can_monitor.h"
#include "app/control.h"
#include "app/imu_monitor.h"
#include "app/motor_monitor.h"
#include "app/serial_tool.h"
#include "base/remote/remote.h"

#ifdef RC_UART
RC rc(RC_UART);
#else
RC rc;
#endif  // RC_UART
#ifdef DEBUG_UART
SerialStudio serial_tool(DEBUG_UART);
#else
SerialStudio serial_tool;
#endif  // DEBUG_UART

BoardComm board_comm(&hcan2);

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

  osThreadDef(serial_tool_task, serialToolTask, osPriorityLow, 0, 1024);
  serialToolTaskHandle = osThreadCreate(osThread(serial_tool_task), NULL);
}
