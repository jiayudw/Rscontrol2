# UART Status

## Current Firmware Mapping

The board connector labeled `UART2` maps to STM32 `USART1`.

Current firmware uses:

```text
STM32 USART1
PA9  = USART1_TX
PB7  = USART1_RX
115200 8N1
```

The UART code path is:

```text
USART1_IRQHandler()
  -> HAL_UART_IRQHandler(&huart1)
  -> HAL_UART_RxCpltCallback()
  -> UartThread_PushByte(rx_byte)
  -> HAL_UART_Receive_IT(&huart1, &rx_byte, 1)
```

Main loop path:

```text
UartThread_Run()
  -> parse binary command frame
  -> update g_debug_target_position / speed / kp / kd / torque
  -> MotorThread_Run10ms()
  -> RS_Motor_BuildControlFrame(...)
```

## Current UART Commands

Supported RX command frames:

```text
0xAA + 27-byte payload + 0x55
0xBB + 27-byte payload + 0x55
```

`0xAA` payload is little-endian:

```text
float32 slot0_position_rad
float32 slot1_position_rad
float32 slot2_position_rad
float32 slot3_position_rad
float32 slot4_position_rad
float32 slot5_position_rad
uint8_t reserved[3]
```

`0xBB` payload is little-endian:

```text
float32 slot6_position_rad
float32 chassis_vx_m_s
float32 chassis_vy_m_s
float32 chassis_wz_rad_s
int8_t lift_command
uint8_t reserved[10]
```

Example:

```bash
python3 -c 'import struct,sys; sys.stdout.buffer.write(b"\xAA" + struct.pack("<ffffff3x", 0.88, 0.0, 0.0, 0.0, 0.0, 0.0) + b"\x55")' > /dev/ttyUSB2
python3 -c 'import struct,sys; sys.stdout.buffer.write(b"\xBB" + struct.pack("<ffffb10x", 0.0, 0.0, 0.0, 0.0, 0) + b"\x55")' > /dev/ttyUSB2
```

Supported control frame:

```text
0xAB + mode + 0x55
```

Modes:

```text
0x00 -> normal position mode
0x01 -> calibration mode
0x02 -> disable UART state telemetry
0x03 -> enable UART state telemetry
```

## Confirmed Working

FT4232H appears as:

```text
/dev/ttyUSB2
/dev/ttyUSB3
/dev/ttyUSB4
/dev/ttyUSB5
```

The active channel receiving board UART output is:

```text
/dev/ttyUSB2
```

Reading `/dev/ttyUSB2` at `115200 8N1` receives:

```text
helloworld
helloworld
```

This confirms:

```text
STM32 USART1 TX -> USB-TTL RX works
```

## Not Working Yet

Sending commands from PC to board has not changed the motor target.

Test command sent:

```bash
python3 -c 'import struct,sys; sys.stdout.buffer.write(b"\xAA" + struct.pack("<ffffff3x", 0.88, 0.0, 0.0, 0.0, 0.0, 0.0) + b"\x55")' > /dev/ttyUSB2
```

GDB result after sending:

```text
g_debug_target_position = 0
huart1.RxState = HAL_UART_STATE_BUSY_RX
huart1.ErrorCode = 0
USART1->SR = 0xc0
USART1->DR = 0x0
```

This means the firmware is still waiting for UART RX, but no received byte is visible in USART1.

## GDB Initialization Check

USART1 initialization checked through GDB:

```text
huart1.Instance = USART1
huart1.Init.BaudRate = 115200
huart1.gState = HAL_UART_STATE_READY
huart1.RxState = HAL_UART_STATE_BUSY_RX
huart1.ErrorCode = 0
USART1->CR1 = 0x202c
GPIOA->AFR[1] includes PA9 as AF7
GPIOB->AFR[0] includes PB7 as AF7
```

Interpretation:

```text
USART1 is enabled.
TX and RX are enabled.
RX interrupt is enabled.
PA9/PB7 are configured as AF7.
HAL_UART_Receive_IT() is armed for 1 byte.
```

## Current Diagnosis

Software initialization looks correct.

Observed direction:

```text
STM32 TX -> PC RX works
```

Unconfirmed / currently failing direction:

```text
PC TX -> STM32 RX
```

The next useful tests are:

```text
1. FTDI loopback on /dev/ttyUSB2 to confirm that channel TX can transmit.
2. GDB breakpoint on HAL_UART_RxCpltCallback() to count whether RX interrupt fires.
3. If loopback works but RX callback never fires, re-check board UART2 RXD pin and FTDI TX wiring/channel.
```

Detailed commands are in `command.md`, section:

```text
UART Loopback And RX Interrupt Test
```
