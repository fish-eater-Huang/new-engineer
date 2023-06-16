/**
 ******************************************************************************
 * @file    referee_protocol.cpp/h
 * @brief   Referee communication(UART) protocol. 裁判系统通信协议
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef REFEREE_PROTOCOL_H
#define REFEREE_PROTOCOL_H

#include <stdint.h>
#include <string.h>

#ifndef __packed
#define __packed __attribute__((packed))
#endif

#define REFEREE_COMM_FRAME_MAX_SIZE 128

const uint8_t referee_comm_sof = 0xA5;

// 裁判系统通信帧头
typedef struct RefereeCommFrameHeader {
  uint8_t sof;
  uint16_t data_len;
  uint8_t seq;
  uint8_t crc8;
} __packed RefereeCommFrameHeader_t;

// 裁判系统通信一帧数据中data以外的内容
typedef struct RefereeCommFrame {
  RefereeCommFrameHeader_t header;
  uint16_t cmd_id;
  // uint8_t data[n];
  uint16_t crc16;
} __packed RefereeCommFrame_t;

// cmd_id
typedef enum RefereeCMDID {
  GAME_STATUS_ID = 0x0001,
  GAME_RESULT_ID = 0x0002,
  GAME_ROBOT_HP_ID = 0x0003,
  DART_STATUS_ID = 0x0004,
  ICRA_BUFF_DEBUFF_ZONE_AND_LURK_STATUS_ID = 0x0005,
  EVENT_DATA_ID = 0x0101,
  SUPPLY_PROJECTILE_ACTION_ID = 0x0102,
  SUPPLY_PROJECTILE_BOOKING_ID = 0x0103,
  REFEREE_WARNING_ID = 0x0104,
  DART_REMAINING_TIME_ID = 0x0105,
  GAME_ROBOT_STATUS_ID = 0x0201,
  POWER_HEAT_DATA_ID = 0x0202,
  GAME_ROBOT_POS_ID = 0x0203,
  BUFF_ID = 0x0204,
  AERIAL_ROBOT_ENERGY_ID = 0x0205,
  ROBOT_HURT_ID = 0x0206,
  SHOOT_DATA_ID = 0x0207,
  BULLET_REMAINING_ID = 0x0208,
  RFID_STATUS_ID = 0x0209,
  DART_CLIENT_CMD_ID = 0X020A,
  STUDENT_INTERACTIVE_DATA_ID = 0x0301,
} RefereeCMDID_e;

// 比赛状态数据：0x0001。发送频率：1Hz
typedef struct {
  uint8_t game_type : 4;
  uint8_t game_progress : 4;
  uint16_t stage_remain_time;
  uint64_t SyncTimeStamp;
} __packed ext_game_status_t;

// 比赛结果数据：0x0002。发送频率：比赛结束后发送
typedef struct {
  uint8_t winner;
} __packed ext_game_result_t;

// 机器人血量数据：0x0003。发送频率：1Hz
typedef struct {
  uint16_t red_1_robot_HP;
  uint16_t red_2_robot_HP;
  uint16_t red_3_robot_HP;
  uint16_t red_4_robot_HP;
  uint16_t red_5_robot_HP;
  uint16_t red_7_robot_HP;
  uint16_t red_outpost_HP;
  uint16_t red_base_HP;
  uint16_t blue_1_robot_HP;
  uint16_t blue_2_robot_HP;
  uint16_t blue_3_robot_HP;
  uint16_t blue_4_robot_HP;
  uint16_t blue_5_robot_HP;
  uint16_t blue_7_robot_HP;
  uint16_t blue_outpost_HP;
  uint16_t blue_base_HP;
} __packed ext_game_robot_HP_t;

// 人工智能挑战赛加成/惩罚区分布与潜伏模式状态：0x0005。发送频率：1Hz周期发送，发送范围：所有机器人
typedef struct {
  uint8_t F1_zone_status : 1;
  uint8_t F1_zone_buff_debuff_status : 3;
  uint8_t F2_zone_status : 1;
  uint8_t F2_zone_buff_debuff_status : 3;
  uint8_t F3_zone_status : 1;
  uint8_t F3_zone_buff_debuff_status : 3;
  uint8_t F4_zone_status : 1;
  uint8_t F4_zone_buff_debuff_status : 3;
  uint8_t F5_zone_status : 1;
  uint8_t F5_zone_buff_debuff_status : 3;
  uint8_t F6_zone_status : 1;
  uint8_t F6_zone_buff_debuff_status : 3;
  uint16_t red1_bullet_left;
  uint16_t red2_bullet_left;
  uint16_t blue1_bullet_left;
  uint16_t blue2_bullet_left;
  uint8_t lurk_mode;
  uint8_t res;
} __packed ext_ICRA_buff_debuff_zone_and_lurk_status_t;

// 场地事件数据：0x0101。发送频率：1Hz
// bit 0-2：
// bit 0：己方补给站 1号补血点占领状态 1为已占领；
// bit 1：己方补给站 2号补血点占领状态 1为已占领；
// bit 2：己方补给站 3号补血点占领状态 1为已占领；
// bit 3-5：己方能量机关状态：
// • bit 3为打击点占领状态，1为占领；
// • bit 4为小能量机关激活状态，1为已激活；
// • bit 5为大能量机关激活状态，1为已激活；
// bit 6：己方侧R2/B2环形高地占领状态1为已占领；
// bit 7：己方侧R3/B3梯形高地占领状态 1为已占领；
// bit 8：己方侧R4/B4梯形高地占领状态 1为已占领；
// bit 9：己方基地护盾状态：
// • 1为基地有虚拟护盾血量；
// • 0为基地无虚拟护盾血量；
// bit 10：己方前哨战状态：
// • 1为前哨战存活；
// • 0为前哨战被击毁；
// bit 10-31: 保留
typedef struct {
  uint32_t event_type;
} __packed ext_event_data_t;

// 补给站动作标识：0x0102。发送频率：动作改变后发送, 发送范围：己方机器人
typedef struct {
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id;
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;
} __packed ext_supply_projectile_action_t;

// 裁判警告信息：cmd_id (0x0104)。发送频率：己方警告发生后发送
typedef struct {
  uint8_t level;
  uint8_t foul_robot_id;
} __packed ext_referee_warning_t;

// 飞镖发射口倒计时：0x0105。发送频率：1Hz周期发送，发送范围：己方机器人
typedef struct {
  uint8_t dart_remaining_time;
} __packed ext_dart_remaining_time_t;

// 比赛机器人状态：0x0201。发送频率：10Hz
typedef struct {
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t remain_HP;
  uint16_t max_HP;
  uint16_t shooter_id1_17mm_cooling_rate;
  uint16_t shooter_id1_17mm_cooling_limit;
  uint16_t shooter_id1_17mm_speed_limit;
  uint16_t shooter_id2_17mm_cooling_rate;
  uint16_t shooter_id2_17mm_cooling_limit;
  uint16_t shooter_id2_17mm_speed_limit;
  uint16_t shooter_id1_42mm_cooling_rate;
  uint16_t shooter_id1_42mm_cooling_limit;
  uint16_t shooter_id1_42mm_speed_limit;
  uint16_t chassis_power_limit;
  uint8_t mains_power_gimbal_output : 1;
  uint8_t mains_power_chassis_output : 1;
  uint8_t mains_power_shooter_output : 1;
} __packed ext_game_robot_status_t;

// 实时功率热量数据：0x0202。发送频率：50Hz
typedef struct {
  uint16_t chassis_volt;
  uint16_t chassis_current;
  float chassis_power;
  uint16_t chassis_power_buffer;
  uint16_t shooter_id1_17mm_cooling_heat;
  uint16_t shooter_id2_17mm_cooling_heat;
  uint16_t shooter_id1_42mm_cooling_heat;
} __packed ext_power_heat_data_t;

// 机器人位置：0x0203。发送频率：10Hz
typedef struct {
  float x;
  float y;
  float z;
  float yaw;
} __packed ext_game_robot_pos_t;

// 机器人增益：0x0204。发送频率：1Hz
// bit 0：机器人血量补血状态
// bit 1：枪口热量冷却加速
// bit 2：机器人防御加成
// bit 3：机器人攻击加成
// 其他bit保留
typedef struct {
  uint8_t power_rune_buff;
} __packed ext_buff_t;

// 空中机器人能量状态：0x0205。发送频率：10Hz
typedef struct {
  uint8_t attack_time;
} __packed ext_aerial_robot_energy_t;

// 伤害状态：0x0206。发送频率：伤害发生后发送
// bit 0-3：
//   当血量变化类型为装甲伤害，代表装甲ID，其中数值为0-4号代表机器人的五个装甲片，其他血量变化类型，该变量数值为0。
// bit 4-7：血量变化类型
//   0x0 装甲伤害扣血；
//   0x1 模块掉线扣血；
//   0x2 超射速扣血；
//   0x3 超枪口热量扣血；
//   0x4 超底盘功率扣血；
//   0x5 装甲撞击扣血
typedef struct {
  uint8_t armor_id : 4;
  uint8_t hurt_type : 4;
} __packed ext_robot_hurt_t;

// 实时射击信息：0x0207。发送频率：射击后发送
typedef struct {
  uint8_t bullet_type;
  uint8_t shooter_id;
  uint8_t bullet_freq;
  float bullet_speed;
} __packed ext_shoot_data_t;

// 子弹剩余发射数：0x0208。发送频率：10Hz周期发送，所有机器人发送
typedef struct {
  uint16_t bullet_remaining_num_17mm;
  uint16_t bullet_remaining_num_42mm;
  uint16_t coin_remaining_num;
} __packed ext_bullet_remaining_t;

// 机器人RFID状态：0x0209。发送频率：1Hz，发送范围：单一机器人
// bit 0：基地增益点RFID状态；
// bit 1：高地增益点RFID状态；
// bit 2：能量机关激活点RFID状态；
// bit 3：飞坡增益点RFID状态；
// bit 4：前哨岗增益点RFID状态；
// bit 6：补血点增益点RFID状态；
// bit 7：工程机器人复活卡RFID状态；
// bit 8-31：保留
typedef struct {
  uint32_t rfid_status;
} __packed ext_rfid_status_t;

// 飞镖机器人客户端指令数据：0x020A。发送频率：10Hz，发送范围：单一机器人
// 当前飞镖发射口的状态
// 1：关闭；
// 2：正在开启或者关闭中
// 0：已经开启
// 飞镖的打击目标，默认为前哨站；
// 0：前哨站；
// 1：基地。
typedef struct {
  uint8_t dart_launch_opening_status;
  uint8_t dart_attack_target;
  uint16_t target_change_time;
  uint16_t operate_launch_cmd_time;
} __packed ext_dart_client_cmd_t;

// 机器人间交互数据

// 交互数据接收信息：0x0301
typedef struct {
  uint16_t data_cmd_id;
  uint16_t sender_ID;
  uint16_t receiver_ID;
} __packed ext_student_interactive_header_data_t;

// 机器人ID（机器人->机器人）
typedef enum RobotID {
  RED_HERO = 0x0001,
  RED_ENGINEER = 0x0002,
  RED_STANDARD3 = 0x0003,
  RED_STANDARD4 = 0x0004,
  RED_STANDARD5 = 0x0005,
  RED_AERIAL = 0x0006,
  RED_SENTRY = 0x0007,
  RED_DART = 0x0008,
  RED_RADAR = 0x0009,
  BLUE_HERO = 0x0101,
  BLUE_ENGINEER = 0x0102,
  BLUE_STANDARD3 = 0x0103,
  BLUE_STANDARD4 = 0x0104,
  BLUE_STANDARD5 = 0x0105,
  BLUE_AERIAL = 0x0106,
  BLUE_SENTRY = 0x0107,
  BLUE_DART = 0x0108,
  BLUE_RADAR = 0x0109,
} RobotID_e;

// 客户端ID（机器人->ui）
typedef enum ClientID {
  RED_HERO_CLIENT = 0x0101,
  RED_ENGINEER_CLIENT = 0x0102,
  RED_STANDARD3_CLIENT = 0x1003,
  RED_STANDARD4_CLIENT = 0x1004,
  RED_STANDARD5_CLIENT = 0x1005,
  RED_AERIAL_CLIENT = 0x0106,
  BLUE_HERO_CLIENT = 0x0165,
  BLUE_ENGINEER_CLIENT = 0x0166,
  BLUE_STANDARD3_CLIENT = 0x0167,
  BLUE_STANDARD4_CLIENT = 0x0168,
  BLUE_STANDARD5_CLIENT = 0x0169,
  BLUE_AERIAL_CLIENT = 0x016A,
} ClientID_e;

// UI

// 客户端删除图形
typedef struct {
  uint8_t operate_tpye;
  uint8_t layer;
} __packed ext_client_custom_graphic_delete_t;

// 图形数据
typedef struct {
  uint8_t graphic_name[3];
  uint32_t operate_tpye : 3;
  uint32_t graphic_tpye : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t start_angle : 9;
  uint32_t end_angle : 9;
  uint32_t width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t radius : 10;
  uint32_t end_x : 11;
  uint32_t end_y : 11;
} graphic_data_struct_t;

// 客户端绘制一个图形
typedef struct {
  graphic_data_struct_t grapic_data_struct;
} __packed ext_client_custom_graphic_single_t;

// 客户端绘制二个图形
typedef struct {
  graphic_data_struct_t grapic_data_struct[2];
} __packed ext_client_custom_graphic_double_t;

// 客户端绘制五个图形
typedef struct {
  graphic_data_struct_t grapic_data_struct[5];
} __packed ext_client_custom_graphic_five_t;

// 客户端绘制七个图形
typedef struct {
  graphic_data_struct_t grapic_data_struct[7];
} __packed ext_client_custom_graphic_seven_t;

// 客户端绘制字符
typedef struct {
  graphic_data_struct_t grapic_data_struct;
  uint8_t data[30];
} __packed ext_client_custom_character_t;

// 小地图下发信息标识：0x0303。发送频率：触发时发送。
typedef struct {
  float target_position_x;
  float target_position_y;
  float target_position_z;
  uint8_t commd_keyboard;
  uint16_t target_robot_ID;
} __packed ext_robot_command_t;

// 小地图接收信息标识：0x0305。最大接收频率：10Hz。
typedef struct {
  uint16_t target_robot_ID;
  float target_position_x;
  float target_position_y;
} __packed ext_client_map_command_t;

#endif  // REFEREE_PROTOCOL_H
