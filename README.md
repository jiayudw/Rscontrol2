# Rscontrol2 使用说明

这是一个基于 STM32F407 的控制固件，目前用于：

- 机械臂 RS 电机控制
- DJI M3508 + C620 麦轮底盘控制
- CAN1 独立 RS 升降台速度控制
- 通过串口接收上位机控制指令

## 当前硬件分配

- 机械臂 RS 电机：`CAN1`，使用扩展帧
- 升降 RS 电机：`CAN1`，使用扩展帧，默认 CAN ID `0x07`
- 底盘 DJI M3508 / C620：`CAN2`，使用标准帧
- 调试/控制串口：`USART1`
- FS-i6/IBUS 遥控器：`USART6`
- 当前 USB-UART 设备：`/dev/ttyUSB0`
- 串口参数：`115200 8N1`，无流控

当前 CAN2 DJI 底盘电机映射定义在：

```text
Core/Src/dji_motor.c
```

默认映射：

```text
index 0 -> LF 左前轮，DJI ID 4
index 1 -> RF 右前轮，DJI ID 1
index 2 -> LB 左后轮，DJI ID 3
index 3 -> RB 右后轮，DJI ID 2
```

如果实际底盘 DJI 电机 ID 不一致，需要修改 `dji_motor.c` 里的 `ids[]`。

当前 CAN1 RS 升降电机配置定义在：

```text
Core/Src/rs_lift_motor.c
```

默认配置：

```text
lift 升降 -> CAN1，CAN ID 0x07
```

注意：CAN1 上机械臂已经使用 `0x01..0x06` 和 `0x7F`，升降电机 ID 不能与这些 ID 冲突。如果实际升降电机 ID 不是 `0x07`，需要修改 `rs_lift_motor.c` 里的 `RS_LIFT_CAN_ID`。

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

### 扩展控制帧

保留旧的 `slot 6`、底盘速度、升降命令字段。

```text
0xBB + 27 字节 payload + 0x55
```

payload：

```text
byte 0-3    float slot6_position_rad
byte 4-7    float vx_m_s
byte 8-11   float vy_m_s
byte 12-15  float wz_rad_s
byte 16     int8  lift_cmd       旧升降台命令字段，当前固件不再执行
byte 17-26  uint8 reserved[10]
```

当前代码状态：

- `slot6_position_rad`：当前不再通过 UART 下发，slot 6 夹爪改由 FS-i6/IBUS CH6 控制
- `vx_m_s` / `vy_m_s` / `wz_rad_s`：当前不再通过 UART 控制底盘，底盘改由 FS-i6/IBUS 摇杆控制
- `lift_cmd`：旧 UART 升降台命令字段，当前已经停用，升降改由 FS-i6/IBUS CH5 控制
- `reserved[10]`：保留给后续夹爪、升降台位置、模式位、急停位等扩展

FS-i6 辅助三段开关 CH5 低档约 `1000` 时输出 `-1` 下降，高档约 `2000` 时输出 `1` 上升，中档约 `1500` 时输出 `0` 停止/保持。RS 升降电机默认目标速度为 `6.283 rad/s`，定义在 `Core/Src/rs_lift_motor.c` 的 `RS_LIFT_TARGET_SPEED_RAD_S`。FS-i6 帧和升降命令都有 `100 ms` 超时保护。

底盘命令有 `100 ms` 超时保护。当前底盘命令来自 FS-i6/IBUS，需要遥控器持续在线。

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

注意：当前 `0xBB` 不再控制底盘速度，底盘由 FS-i6/IBUS 控制。

## 机械臂机械零点/编码校准

当前 RS 电机的机械零点使用固定表，定义在：

```text
Core/Src/motor_thread.c
```

变量：

```c
static const float motor_startq_raw_rad[MOTOR_SLOT_COUNT] = {
    3.541f,
    4.447f,
    3.740f,
    2.152f,
    2.960f,
    2.830f,
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

### CAN2 DJI 底盘电机

```text
dji_motors
```

当前 CAN2 DJI 底盘电机映射：

```text
dji_motors[0] -> LF 左前轮，ID 4
dji_motors[1] -> RF 右前轮，ID 1
dji_motors[2] -> LB 左后轮，ID 3
dji_motors[3] -> RB 右后轮，ID 2
```

常看字段：

```text
dji_motors[i].id
dji_motors[i].direction
dji_motors[i].ecd
dji_motors[i].speed_rpm
dji_motors[i].current_raw
dji_motors[i].temperature
dji_motors[i].target_rpm
dji_motors[i].output_current
dji_motors[i].online
dji_motors[i].last_update_ms
```

说明：

- `target_rpm`：固件给该轮子的目标转速，单位 rpm
- `speed_rpm`：C620 回传的实际转速，单位 rpm
- `output_current`：固件最终下发给 C620 的电流命令
- `current_raw`：C620 回传电流原始值
- `online`：是否收到过该电机反馈

停止后建议确认：

```text
dji_motors[i].target_rpm == 0
dji_motors[i].output_current == 0
g_chassis_debug_target_rpm[i] == 0
```

### CAN1 RS 升降电机

```text
rs_lift_motor
```

默认配置：

```text
CAN1，CAN ID 0x07
```

常看字段：

```text
rs_lift_motor.id
rs_lift_motor.target_speed_rad_s
rs_lift_motor.position_rad
rs_lift_motor.velocity_rad_s
rs_lift_motor.torque_nm
rs_lift_motor.temperature_c
rs_lift_motor.online
rs_lift_motor.last_update_ms
```

相关调试变量：

```text
g_rs_lift_cmd
g_rs_lift_online
g_rs_lift_control_count
g_rs_lift_feedback_count
g_rs_lift_timeout_count
g_rs_lift_last_tx_status
g_rs_lift_last_tx_error
```

## 未来规划

### 上位机协议扩展

当前 `0xBB` 底盘帧保留旧 `lift_cmd` 字段，并预留了 `reserved[10]`，后续可以扩展：

```text
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

当前升降台已经通过 FS-i6/IBUS CH5 控制 CAN1 RS ID `0x07` 做速度控制，后续还需要补：

- 限位开关/零位检测
- 位置闭环
- 上电回零流程
- 急停处理

### 底盘 telemetry

当前串口 telemetry 会回传机械臂 RS 电机、CAN2 DJI 底盘和 CAN1 RS 升降状态，例如：

```text
dji index id online target_rpm speed_rpm current out temp
rs_lift id online target_mrad_s speed_mrad_s raw_mrad tau_mNm temp_c10
```

这样不用每次都通过 Ozone/GDB 看 `dji_motors` 和 `rs_lift_motor`。


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

当前仓库也提供 CMake preset，推荐使用：

```bash
cmake --build --preset Debug
```

J-Link 烧录：

```bash
JLinkExe -device STM32F407IG -if SWD -speed 1000 -autoconnect 1 -CommanderScript flash.jlink
```



10 0.3
50 0.2
5 0.2
10 0.15
20 0.5
5 0.2
