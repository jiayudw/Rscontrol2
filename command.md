# Rscontrol2 Commands

## Build

```bash
cmake --build --preset Debug
```

## Flash With J-Link

```bash
JLinkExe -device STM32F407IG -if SWD -speed 4000 -autoconnect 1 -CommanderScript flash.jlink
```

## Start J-Link GDB Server

Run this in one terminal and keep it open:

```bash
JLinkGDBServer -device STM32F407IG -if SWD -speed 4000
```

## Change Target Position With GDB

Run this in another terminal:

```bash
gdb-multiarch build/Debug/Rscontrol2.elf
```

Inside GDB:

```gdb
target remote localhost:2331
set var g_debug_target_position = 0.8f
set var g_debug_kp = 0.5f
set var g_debug_kd = 0.01f
continue
```

Or run the prepared script:

```bash
gdb-multiarch build/Debug/Rscontrol2.elf -x debug_target.gdb
```

If you are already inside GDB and see `(gdb)`, connect only once:

```gdb
target remote localhost:2331
```

After it connects, do not run `target remote localhost:2331` again. Change values with:

```gdb
set var g_debug_target_position = 2f
set var g_debug_kp = 0.5f
set var g_debug_kd = 0.01f
continue
```

If multiline paste is parsed incorrectly, use the prepared variable script after connecting:

```gdb
source set_debug_vars.gdb
```

If GDB is already confused, quit and restart clean:

```gdb
quit
y
```

Then run:

```bash
gdb-multiarch build/Debug/Rscontrol2.elf -x debug_target.gdb
```

To change position again while the firmware is running, press `Ctrl-C` in GDB, then:

```gdb
set var g_debug_target_position = -0.8f
continue
```

Useful debug variables:

```gdb
set var g_debug_target_position = 0.0f
set var g_debug_target_position = 0.8f
set var g_debug_target_position = -0.8f
set var g_debug_kp = 0.5f
set var g_debug_kd = 0.5f
set var g_debug_target_speed = 0.0f
set var g_debug_target_torque = 0.0f
```

## Inspect Motor State With GDB

The current firmware stores feedback in `motor_states[id]`. The current test motor ID is `0x7F`, so use index `127`:

```gdb
print motor_states[127]
print motor_states[127].motor_position
print motor_states[127].motor_velocity
print motor_states[127].motor_torque
print motor_states[127].temperature
print motor_states[127].joint_position
print motor_states[127].mode_state
print motor_states[127].fault_code
print motor_states[127].is_valid
print motor_states[127].last_update_ms
```

You can change calibration mapping live:

```gdb
set variable motor_states[127].direction = -1.0
set variable motor_states[127].offset = 0.2
```

## UART Debug Commands

Board connector `UART2` maps to STM32 `USART1`.

USART1 is configured as `115200 8N1` on `PA9=TX`, `PB7=RX`.

Send one fixed-length binary command frame:

```text
0xAA + 24-byte payload + 0x55
```

Payload is little-endian:

```text
uint32 index
float32 target_position
float32 target_speed
float32 kp
float32 kd
float32 target_torque
```

Example: set slot 0 to position `0.8` rad:

```bash
python3 -c 'import struct,sys; sys.stdout.buffer.write(b"\xAA" + struct.pack("<Ifffff", 0, 0.8, 0.0, 0.5, 0.01, 0.0) + b"\x55")' > "$TTY"
```

The board still transmits motor state as ASCII `state ...` lines.

For slot 0 position tests from `-3` to `3` rad:

```bash
export TTY=/dev/ttyUSB2
stty -F "$TTY" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw

for p in -3.000 -2.333 -1.667 -1.000 -0.333 0.333 1.000 1.667 2.333 3.000; do
  python3 -c 'import struct,sys; p=float(sys.argv[1]); sys.stdout.buffer.write(b"\xAA" + struct.pack("<Ifffff", 0, p, 0.0, 0.5, 0.01, 0.0) + b"\x55")' "$p" > "$TTY"
  sleep 0.2
done
```

## UART Loopback And RX Interrupt Test

Current confirmed board mapping:

```text
Board connector UART2 -> STM32 USART1
USART1 TX = PA9
USART1 RX = PB7
Baudrate = 115200 8N1
```

The FT4232H channel that receives `helloworld` is currently:

```bash
export TTY=/dev/ttyUSB2
```

Configure the port:

```bash
stty -F "$TTY" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw
```

### 1. Confirm Board TX

This checks STM32 TX to USB-TTL RX:

```bash
timeout 3s cat "$TTY"
```

Expected output:

```text
helloworld
helloworld
```

### 2. FTDI Channel Loopback Test

Disconnect the FTDI channel from the board. Short the same FTDI channel's TX and RX pins together, then run:

```bash
rm -f /tmp/uart_loopback.txt
stty -F "$TTY" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw
timeout 3s sh -c "cat '$TTY' > /tmp/uart_loopback.txt" &
sleep 0.2
printf "looptest\n" > "$TTY"
wait
cat /tmp/uart_loopback.txt
```

Expected output:

```text
looptest
```

If this fails, the PC-side TX/RX channel or wiring is wrong.

### 3. Send A Position Command To The Board

Reconnect FTDI to the board:

```text
FTDI TX -> Board UART2 RXD / Pin1
FTDI RX -> Board UART2 TXD / Pin2
FTDI GND -> Board UART2 GND / Pin3
```

Then send:

```bash
stty -F "$TTY" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw
python3 -c 'import struct,sys; sys.stdout.buffer.write(b"\xAA" + struct.pack("<Ifffff", 0, 0.88, 0.0, 0.5, 0.01, 0.0) + b"\x55")' > "$TTY"
```

Check with GDB:

```bash
JLinkGDBServer -device STM32F407IG -if SWD -speed 4000
```

In another terminal:

```bash
gdb-multiarch build/Debug/Rscontrol2.elf
```

Inside GDB:

```gdb
target remote localhost:2331
print g_debug_target_position
print huart1.RxState
print huart1.ErrorCode
print/x USART1->SR
detach
quit
```

Expected after a valid binary command frame:

```text
g_debug_target_position = 0.88
huart1.ErrorCode = 0
```

### 4. Count RX Interrupt Hits With GDB

This checks whether `HAL_UART_RxCpltCallback()` is triggered. This is a GDB session counter, not a firmware variable.

Terminal 1:

```bash
JLinkGDBServer -device STM32F407IG -if SWD -speed 4000
```

Terminal 2:

```bash
gdb-multiarch build/Debug/Rscontrol2.elf
```

Inside GDB:

```gdb
target remote localhost:2331
set $rx_hits = 0
break HAL_UART_RxCpltCallback
commands
silent
set $rx_hits = $rx_hits + 1
continue
end
continue
```

Terminal 3, send bytes:

```bash
export TTY=/dev/ttyUSB2
stty -F "$TTY" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw
python3 -c 'import struct,sys; sys.stdout.buffer.write(b"\xAA" + struct.pack("<Ifffff", 0, 0.88, 0.0, 0.5, 0.01, 0.0) + b"\x55")' > "$TTY"
```

Back in GDB, press `Ctrl-C`, then:

```gdb
print $rx_hits
print g_debug_target_position
continue
```

Expected if board RX works:

```text
$rx_hits > 0
g_debug_target_position = 0.88
```
