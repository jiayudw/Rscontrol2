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

The current firmware stores feedback by slot index, not by CAN ID. Slot 0 currently maps to CAN ID `0x7F` / `127`, so inspect `g_motor_states[0]`:

```gdb
print g_motor_states[0]
print g_motor_states[0].motor_position
print g_motor_states[0].motor_velocity
print g_motor_states[0].motor_torque
print g_motor_states[0].temperature
print g_motor_states[0].joint_position
print g_motor_states[0].mode_state
print g_motor_states[0].fault_code
print g_motor_states[0].is_valid
print g_motor_states[0].last_update_ms
```

You can inspect the active zero offset:

```gdb
print g_motor_zero_offsets[0]
print g_motor_calibration_mode
```

Slot 0 / CAN ID `127` currently uses a Startq-style raw zero offset:

```text
g_motor_zero_offsets[0] = 2.487 rad
joint_position = raw_position - 2.487
raw_command = joint_command + 2.487
```

Edit all seven Startq-style raw zero offsets in `Core/Src/motor_thread.c`:

```c
static const float motor_startq_raw_rad[MOTOR_SLOT_COUNT] = {
    2.487f, // slot 0, CAN ID 127
    0.0f,  // slot 1, CAN ID 1
    0.0f,  // slot 2, CAN ID 2
    0.0f,  // slot 3, CAN ID 3
    0.0f,  // slot 4, CAN ID 4
    0.0f,  // slot 5, CAN ID 5
    0.0f,  // slot 6, CAN ID 6
};
```

## UART Debug Commands

Board connector `UART2` maps to STM32 `USART1`.

USART1 is configured as `115200 8N1` on `PA9=TX`, `PB7=RX`.

The upper computer now sends two fixed-length 29-byte binary command frames.

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

Current slot-to-CAN-ID mapping:

```text
slot 0 -> CAN ID 127 / 0x7F
slot 1 -> CAN ID 1
slot 2 -> CAN ID 2
slot 3 -> CAN ID 3
slot 4 -> CAN ID 4
slot 5 -> CAN ID 5
slot 6 -> CAN ID 6
```

The firmware applies fixed control parameters to all received arm positions:

```text
speed = 0.0 rad/s
kp = 1.1
kd = 0.1
torque = 0.0 Nm
```

Configure the UART port:

```bash
export TTY=/dev/ttyUSB2
stty -F "$TTY" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw
```

Example: set only slot 0 / CAN ID 127 to position `0.8` rad, and command slots 1-5 to `0.0` rad:

```bash
python3 -c 'import struct,sys; p0=float(sys.argv[1]); sys.stdout.buffer.write(b"\xAA" + struct.pack("<ffffff3x", p0, 0.0, 0.0, 0.0, 0.0, 0.0) + b"\x55")' 0.8 > "$TTY"
```

Example: command all seven slots plus chassis velocity:

```bash
python3 -c 'import struct,sys; vals=[float(v) for v in sys.argv[1:7]]; sys.stdout.buffer.write(b"\xAA" + struct.pack("<ffffff3x", *vals) + b"\x55")' \
  0.8 0.1 0.2 0.3 0.4 0.5 > "$TTY"
python3 -c 'import struct,sys; p6,vx,vy,wz=[float(v) for v in sys.argv[1:5]]; sys.stdout.buffer.write(b"\xBB" + struct.pack("<ffffb10x", p6, vx, vy, wz, 0) + b"\x55")' \
  0.6 0.0 0.0 0.0 > "$TTY"
```

Example: return slots 0-5 to zero:

```bash
python3 -c 'import struct,sys; sys.stdout.buffer.write(b"\xAA" + struct.pack("<ffffff3x", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) + b"\x55")' > "$TTY"
```

Example: sweep slot 0 / CAN ID 127 from `-1.0` to `1.0` rad while holding slots 1-5 at zero:

```bash
for p in -1.000 -0.750 -0.500 -0.250 0.000 0.250 0.500 0.750 1.000; do
  python3 -c 'import struct,sys; p0=float(sys.argv[1]); sys.stdout.buffer.write(b"\xAA" + struct.pack("<ffffff3x", p0, 0.0, 0.0, 0.0, 0.0, 0.0) + b"\x55")' "$p" > "$TTY"
  sleep 0.2
done
```

Example: send a smooth sine command to slot 0 / CAN ID 127 at 50 Hz:

```bash
python3 - <<'PY' > "$TTY"
import math
import struct
import sys
import time

for step in range(250):
    p0 = 0.5 * math.sin(step * 0.04)
    sys.stdout.buffer.write(b"\xAA" + struct.pack("<ffffff3x", p0, 0.0, 0.0, 0.0, 0.0, 0.0) + b"\x55")
    sys.stdout.buffer.flush()
    time.sleep(0.02)
PY
```

Motor state telemetry is disabled by default. The monitor script enables it while running and disables it on exit, then parses ASCII `state ...` lines from the board.

```bash
python3 tools/parse_motor_state.py "$TTY" --raw
```

## UART Calibration Mode

Calibration mode uses a separate short control frame:

```text
0xAB + mode + 0x55
```

Modes:

```text
0x01 -> enter calibration mode
0x00 -> return to normal position mode
0x03 -> enable UART state telemetry
0x02 -> disable UART state telemetry
```

Slot 0 / CAN ID `127` uses a fixed Startq-style raw zero offset of `2.487 rad`. Calibration mode does not overwrite that offset; it is only a zero-force raw-data monitor. While calibration mode is active:

```text
commanded joint position = 0.0
commanded kp = 0.0
commanded kd = 0.0
commanded torque = 0.0
reported p_mrad = current joint position from the STM32
reported raw_mrad = current raw motor position
mode = 1
```

Enter zero-force calibration mode and watch raw data on `/dev/ttyUSB2`:

```bash
export TTY=/dev/ttyUSB0
python3 tools/parse_motor_state.py "$TTY" --enter-calibration --raw-table
```

The command above sends `0xAB 0x01 0x55`, then opens a live raw-position monitor. In calibration mode, `joint_rad` is parsed from the STM32 `p_mrad` field, `raw_rad` shows the current raw motor position, and the motor command uses zero force gains.

Watch raw state lines instead of the table:

```bash
python3 tools/parse_motor_state.py "$TTY" --enter-calibration --raw
```

Expected state lines in calibration mode:

```text
state 0 id 127 en 1 valid 1 mode 1 p_mrad <current_joint> raw_mrad <current_raw> ...
```

Return to normal position mode:

```bash
python3 tools/parse_motor_state.py "$TTY" --exit-calibration --send-only
```

After returning to normal mode, slot 0 commands are interpreted relative to the fixed `2.487 rad` Startq offset:

```bash
python3 -c 'import struct,sys; sys.stdout.buffer.write(b"\xAA" + struct.pack("<ffffff3x", 0.5, 0.0, 0.0, 0.0, 0.0, 0.0) + b"\x55")' > "$TTY"
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
python3 -c 'import struct,sys; sys.stdout.buffer.write(b"\xAA" + struct.pack("<ffffff3x", 0.88, 0.0, 0.0, 0.0, 0.0, 0.0) + b"\x55")' > "$TTY"
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
python3 -c 'import struct,sys; sys.stdout.buffer.write(b"\xAA" + struct.pack("<ffffff3x", 0.88, 0.0, 0.0, 0.0, 0.0, 0.0) + b"\x55")' > "$TTY"
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
