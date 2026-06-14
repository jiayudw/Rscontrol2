# Rscontrol2 使用说明

这是一个基于 STM32F407 的控制固件，目前用于：

- 机械臂 RS 电机控制
- DJI M3508 + C620 麦轮底盘控制
- 预留升降台/扩展执行机构控制字段
- 通过串口接收上位机控制指令

## 当前硬件分配

- 机械臂 RS 电机：`CAN1`，使用扩展帧
- 底盘 DJI M3508 / C620：`CAN2`，使用标准帧
- 调试/控制串口：`USART1`
- 当前 USB-UART 设备：`/dev/ttyUSB0`
- 串口参数：`115200 8N1`，无流控

## 串口协议

所有多字节数据都是 little-endian。

### 机械臂位置帧

用于控制机械臂 `slot 0..5`。

```text
0xAA + 27 字节 payload + 0x55
```

payload：

```text
float arm0_position_rad
float arm1_position_rad
float arm2_position_rad
float arm3_position_rad
float arm4_position_rad
float arm5_position_rad
uint8 reserved[3]
```

### 底盘控制帧

用于控制机械臂 `slot 6`、底盘速度，以及预留的升降台命令。

```text
0xBB + 27 字节 payload + 0x55
```

payload：

```text
byte 0-3    float slot6_position_rad
byte 4-7    float vx_m_s
byte 8-11   float vy_m_s
byte 12-15  float wz_rad_s
byte 16     int8  lift_cmd
byte 17-26  uint8 reserved[10]
```

当前底盘速度方向约定：

- `vx > 0`：前进
- `vy > 0`：右移
- `wz > 0`：逆时针旋转

当前代码状态：

- `slot6_position_rad`：已经接入，会写入机械臂 `slot 6` 目标位置
- `vx_m_s` / `vy_m_s` / `wz_rad_s`：已经接入，会控制麦轮底盘
- `lift_cmd`：协议已预留，当前固件只读取占位，还没有驱动升降台
- `reserved[10]`：保留给后续夹爪、升降台位置、模式位、急停位等扩展

底盘命令有 `100 ms` 超时保护。如果要持续运动，需要上位机周期发送底盘帧，建议 `20 ms` 到 `50 ms` 发一次。

### 控制帧

```text
0xAB + 1 字节 mode + 0x55
```

当前 mode：

```text
0x00 退出机械臂校准模式
0x01 进入机械臂校准模式
0x02 关闭 telemetry 回传
0x03 打开 telemetry 回传
```

## CuteCom 快速测试

CuteCom 参数：

```text
Port: /dev/ttyUSB0
Baud: 115200
Data: 8
Stop: 1
Parity: none
Flow control: none
Input mode: Hex
```

打开 telemetry：

```text
AB 03 55
```

关闭 telemetry：

```text
AB 02 55
```

底盘停止帧：

```text
BB 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55
```

底盘前进 `0.3 m/s`：

```text
BB 00 00 00 00 9A 99 99 3E 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55
```

注意：只发送一次运动帧，底盘会因为 `100 ms` 超时保护很快停下。要持续运动，需要循环发送运动帧。停止时发送停止帧。

## 机械臂机械零点/编码校准

当前 RS 电机的机械零点使用固定表，定义在：

```text
Core/Src/motor_thread.c
```

变量：

```c
static const float motor_startq_raw_rad[MOTOR_SLOT_COUNT] = {
    2.487f,
    4.439f,
    3.741f,
    2.155f,
    -7.015f,
    0.0f,
    0.0f,
};
```

含义：

- 数组下标就是 `slot index`
- 数值是该关节处于机械零位时，电机反馈的原始角度，单位 rad
- 固件运行时不会自动保存零点；校准后需要手动修改这个表、重新编译并烧录

### 校准流程

1. 打开串口 telemetry：

```text
AB 03 55
```

2. 进入零力校准模式：

```text
AB 01 55
```

进入校准模式后，固件会给 RS 电机下发：

```text
target_position = 0
kp = 0
kd = 0
torque = 0
```

这时电机基本不主动出力，方便手动转动关节。

3. 手动把某个关节摆到机械零位。

4. 在 CuteCom 或串口工具里查看对应 slot 的 telemetry：

```text
state <slot> id <can_id> en <...> valid <...> mode 1 p_mrad <...> raw_mrad <...>
```

重点看：

```text
raw_mrad
```

把 `raw_mrad` 除以 `1000`，得到单位 rad 的原始机械零位。

例如：

```text
raw_mrad 4439
```

对应：

```text
4.439f
```

5. 把读到的值写入 `motor_startq_raw_rad[]` 对应 slot。

例如校准 `slot 1`：

```c
static const float motor_startq_raw_rad[MOTOR_SLOT_COUNT] = {
    2.487f,
    4.439f, // slot 1
    ...
};
```

6. 退出校准模式：

```text
AB 00 55
```

7. 重新编译并烧录。

### 校准后检查

烧录后，把关节摆回机械零位，打开 telemetry，确认：

```text
p_mrad 接近 0
raw_mrad 接近刚才写入的零点值
```

说明：

- `raw_mrad` 是电机原始角度
- `p_mrad` 是扣除 `motor_startq_raw_rad[]` 后的关节角度
- 如果机械零位处 `p_mrad` 不是 0，说明对应 slot 的 offset 还需要修正

## Ozone 常用观察变量

### 机械臂命令

```text
g_motor_commands
```

每个 slot 的目标命令：

```text
g_motor_commands[i].target_position
g_motor_commands[i].target_speed
g_motor_commands[i].target_torque
g_motor_commands[i].kp
g_motor_commands[i].kd
g_motor_commands[i].enabled
```

### 机械臂反馈

```text
g_motor_states
```

常看字段：

```text
g_motor_states[i].can_id
g_motor_states[i].is_valid
g_motor_states[i].is_enabled
g_motor_states[i].joint_position
g_motor_states[i].joint_velocity
g_motor_states[i].motor_position
g_motor_states[i].motor_velocity
g_motor_states[i].motor_torque
g_motor_states[i].temperature
g_motor_states[i].last_update_ms
```

说明：

- `joint_position`：应用零点和方向后的关节角度，单位 rad
- `motor_position`：电机原始角度，单位 rad
- `joint_velocity`：关节速度
- `is_valid`：是否收到过有效反馈

### 机械臂校准/零点

```text
g_motor_calibration_mode
g_motor_zero_offsets
```

调试 slot 0 时也可以看：

```text
g_debug_target_position
g_debug_target_speed
g_debug_target_torque
g_debug_kp
g_debug_kd
```

### 底盘命令

```text
chassis_command
```

常看字段：

```text
chassis_command.vx
chassis_command.vy
chassis_command.wz
chassis_command.enabled
chassis_command.last_update_ms
```

说明：

- `vx`：前后速度，单位 m/s
- `vy`：左右速度，单位 m/s
- `wz`：旋转速度，单位 rad/s
- `last_update_ms`：最后一次收到有效底盘帧的时间

### DJI M3508 底盘电机

```text
dji_motors
```

当前底盘电机映射：

```text
dji_motors[0] -> LF，ID 1
dji_motors[1] -> RF，ID 2
dji_motors[2] -> LB，ID 4
dji_motors[3] -> RB，ID 3
```

常看字段：

```text
dji_motors[i].id
dji_motors[i].direction
dji_motors[i].ecd
dji_motors[i].last_ecd
dji_motors[i].speed_rpm
dji_motors[i].current_raw
dji_motors[i].temperature
dji_motors[i].round_count
dji_motors[i].total_angle_deg
dji_motors[i].target_rpm
dji_motors[i].output_current
dji_motors[i].online
dji_motors[i].last_update_ms
```

说明：

- `ecd`：C620 回传的 0 到 8191 编码器值
- `speed_rpm`：C620 回传转速，单位 rpm
- `current_raw`：C620 回传电流原始值
- `target_rpm`：固件给该轮子的目标转速
- `output_current`：固件最终下发给 C620 的电流命令
- `online`：是否收到过该电机反馈
- `total_angle_deg`：根据编码器累计得到的多圈角度，单位 degree

停止后建议确认：

```text
dji_motors[i].target_rpm == 0
dji_motors[i].output_current == 0
dji_motors[i].speed_pid.integral == 0
```

## 未来规划

### 上位机协议扩展

当前 `0xBB` 底盘帧已经预留了 `lift_cmd` 和 `reserved[10]`，后续可以扩展：

```text
lift_cmd              升降台方向命令，例如 -1/0/+1
lift_target_position  升降台目标位置
gripper_cmd           夹爪开合命令
mode_bits             底盘/机械臂/升降台模式位
estop                 急停位
checksum              简单校验或 CRC
```

如果控制量继续变多，建议把协议升级成：

```text
帧头 + msg_id + payload_len + payload + checksum + 帧尾
```

这样机械臂、底盘、升降台、夹爪可以用不同 `msg_id` 分发，避免固定 27 字节 payload 越来越难维护。

### 升降台

当前 `lift_cmd` 只是协议占位，后续需要补：

- 升降台电机驱动模块
- 限位开关/零位检测
- 位置或速度闭环
- 上电回零流程
- 超时保护和急停处理

### 底盘 telemetry

当前串口 telemetry 主要回传机械臂 RS 电机状态。后续建议增加底盘状态回传，例如：

```text
dji_state index id online ecd speed_rpm target_rpm output_current temp
```

这样不用每次都通过 Ozone/GDB 看 `dji_motors`。


### 安全保护

后续建议补充：

- 全局急停位
- 底盘/机械臂/升降台独立使能位
- 串口控制帧 checksum
- CAN 反馈超时报警
- 底盘电流/速度限幅参数可配置

## 编译和烧录

编译：

```bash
cmake --build build/Debug
```

J-Link 烧录：

```bash
JLinkExe -device STM32F407IG -if SWD -speed 4000 -CommanderScript flash.jlink
```
