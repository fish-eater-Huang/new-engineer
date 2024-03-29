# 新工程操作键位

## **1 比赛键位**

遥控器右1

### **底盘控制**

| 键位            | 功能                   |
| --------------- | ---------------------- |
| wasd            | 底盘移动               |
| ctrl/shift+wasd | 慢速/快速移动          |
| q               | 陀螺                   |
| ctrl+q          | 关闭陀螺，不影响机械臂 |
| f               | 关闭陀螺，收回机械臂   |
| 鼠标移动        | J0+云台pitch，底盘跟随 |
| 鼠标左键        | 云台pitch回零          |

### **机械臂控制**

**1 遥控器**

| 遥控器挡位 | 左1右1 | 左2右1 | 左3右1（自定义控制器） |
| ---------- | ------ | ------ | ---------------------- |
| 左摇杆x    | x      | x      | x                      |
| 左摇杆y    | y      | y      | y                      |
| 拨轮       | roll   | z      | z                      |
| 右摇杆x    | yaw    | yaw    | 保留（气泵/任务等）    |
| 右摇杆y    | pitch  | pitch  | 保留（气泵/任务等）    |

**2 手控**（自定义控制器，imu×3+主控×1）

遥控器左3右1，在线-接图传链路，离线-UART线直连

### **任务**

| 键位         | 功能                     |
| ------------ | ------------------------ |
| F            | 进入行驶模式(机械臂收回) |
| ctrl+F       | 进入行驶模式(机械臂伸出) |
| E            | 常规取矿位姿             |
| shift+E      | 空接取矿位姿             |
| ctrl+E       | 地面取矿位姿             |
| R/shift+R    | 兑换位姿/推入            |
| C/ctrl+C     | 左存矿/出矿              |
| V/ctrl+V     | 右存矿/出矿              |
| X/ctrl+X     | 开始取矿                 |
| Z/shift+X    | 三连                     |
| G            | 关闭机械臂气泵           |
| ctrl+shift+B | 关闭存矿气泵             |

## **2 遥控器测试键位**

机械臂控制可直接使用比赛键位（即遥控器右2档）进行测试。

| 遥控器挡位 | 左1右2（移动） | 左2右2                   | 左3右2         |
| ---------- | -------------- | ------------------------ | -------------- |
| 左摇杆x    | yaw            | 左/右存矿                | 机械臂气泵舵机 |
| 左摇杆y    | pitch          | +：取矿位姿，-：开始取矿 | 机械臂气泵电机 |
| 右摇杆x    | vx             | 左/右出矿                | 存矿气泵舵机   |
| 右摇杆y    | vy             | +：兑换位姿，-：开始兑换 | 存矿气泵电机   |
| 拨轮       | 陀螺           | 暂停/复位                |                |
