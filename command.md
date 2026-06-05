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
