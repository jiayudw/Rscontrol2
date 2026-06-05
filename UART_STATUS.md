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
  -> parse newline-terminated ASCII command
  -> update g_debug_target_position / speed / kp / kd / torque
  -> MotorThread_Run10ms()
  -> RS05_Private_Control(...)
```

## Current UART Commands

Supported newline-terminated commands:

```text
pos 0.8
kp 0.5
kd 0.01
speed 0.0
torque 0.0
cmd 0.8 0.0 0.5 0.01 0.0
```

`cmd` field order:

```text
target_position target_speed kp kd target_torque
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
printf "pos 0.88\n" > /dev/ttyUSB2
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
