#!/usr/bin/env python3
"""Read and display motor state lines from the STM32 UART."""

import argparse
import os
import re
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass


STATE_RE = re.compile(
    r"^state\s+(?P<index>\d+)\s+"
    r"id\s+(?P<can_id>\d+)\s+"
    r"en\s+(?P<enabled>\d+)\s+"
    r"valid\s+(?P<valid>\d+)\s+"
    r"p_mrad\s+(?P<p_mrad>-?\d+)\s+"
    r"v_mrad_s\s+(?P<v_mrad_s>-?\d+)\s+"
    r"tau_mNm\s+(?P<tau_mNm>-?\d+)\s+"
    r"temp_c10\s+(?P<temp_c10>-?\d+)"
)


BAUD_RATES = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
    921600: termios.B921600,
}


@dataclass
class MotorState:
    index: int
    can_id: int
    enabled: int
    valid: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    temperature_c: float
    update_count: int = 0
    update_hz: float = 0.0
    last_update: float = 0.0


def configure_serial(fd: int, baud: int) -> None:
    if baud not in BAUD_RATES:
        supported = ", ".join(str(rate) for rate in sorted(BAUD_RATES))
        raise ValueError(f"Unsupported baud {baud}; supported: {supported}")

    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = BAUD_RATES[baud]
    attrs[5] = BAUD_RATES[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    tty.setraw(fd)
    termios.tcflush(fd, termios.TCIOFLUSH)


def parse_state_line(line: str, now: float, previous: MotorState | None) -> MotorState | None:
    match = STATE_RE.match(line.strip())
    if not match:
        return None

    values = {key: int(value) for key, value in match.groupdict().items()}
    update_count = 1
    update_hz = 0.0

    if previous is not None:
        update_count = previous.update_count + 1
        dt = now - previous.last_update
        if dt > 0.0:
            update_hz = 1.0 / dt

    return MotorState(
        index=values["index"],
        can_id=values["can_id"],
        enabled=values["enabled"],
        valid=values["valid"],
        position_rad=values["p_mrad"] / 1000.0,
        velocity_rad_s=values["v_mrad_s"] / 1000.0,
        torque_nm=values["tau_mNm"] / 1000.0,
        temperature_c=values["temp_c10"] / 10.0,
        update_count=update_count,
        update_hz=update_hz,
        last_update=now,
    )


def render_table(states: dict[int, MotorState], parsed_count: int, bad_count: int) -> None:
    now = time.monotonic()
    print("\033[2J\033[H", end="")
    print("Rscontrol2 motor state monitor")
    print(f"parsed_lines={parsed_count} unparsed_lines={bad_count} time={time.strftime('%H:%M:%S')}")
    print()
    print("idx  id   en  valid  age_ms  hz     pos_rad    vel_rad_s  torque_Nm  temp_C")
    print("---  ---  --  -----  ------  -----  ---------  ---------  ---------  ------")

    for index in range(7):
        state = states.get(index)
        if state is None:
            print(f"{index:>3}  ---  --  -----  ------  -----  ---------  ---------  ---------  ------")
            continue

        age_ms = int((now - state.last_update) * 1000.0)
        print(
            f"{state.index:>3}  "
            f"{state.can_id:>3}  "
            f"{state.enabled:>2}  "
            f"{state.valid:>5}  "
            f"{age_ms:>6}  "
            f"{state.update_hz:>5.1f}  "
            f"{state.position_rad:>9.3f}  "
            f"{state.velocity_rad_s:>9.3f}  "
            f"{state.torque_nm:>9.3f}  "
            f"{state.temperature_c:>6.1f}"
        )

    print()
    print("Ctrl-C to quit.")
    sys.stdout.flush()


def previous_state_for_line(line: str, states: dict[int, MotorState]) -> MotorState | None:
    parts = line.split()
    if len(parts) < 2 or parts[0] != "state" or not parts[1].isdigit():
        return None
    return states.get(int(parts[1]))


def main() -> int:
    parser = argparse.ArgumentParser(description="Parse Rscontrol2 UART motor state output.")
    parser.add_argument("port", nargs="?", default="/dev/ttyUSB2", help="UART device, default: /dev/ttyUSB2")
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate, default: 115200")
    parser.add_argument("--refresh", type=float, default=0.2, help="screen refresh interval in seconds")
    parser.add_argument("--raw", action="store_true", help="print raw parsed state lines instead of the table")
    args = parser.parse_args()

    states: dict[int, MotorState] = {}
    line_buffer = bytearray()
    parsed_count = 0
    bad_count = 0
    last_render = 0.0

    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_serial(fd, args.baud)
        print(f"Reading {args.port} at {args.baud} baud. Ctrl-C to quit.")

        while True:
            readable, _, _ = select.select([fd], [], [], 0.05)
            if readable:
                chunk = os.read(fd, 4096)
                for byte in chunk:
                    if byte == 10:
                        line = line_buffer.decode("ascii", errors="replace").strip()
                        line_buffer.clear()
                        if not line:
                            continue

                        now = time.monotonic()
                        state = parse_state_line(line, now, previous_state_for_line(line, states))
                        if state is None:
                            bad_count += 1
                            if args.raw:
                                print(f"unparsed: {line}")
                            continue

                        states[state.index] = state
                        parsed_count += 1
                        if args.raw:
                            print(line)
                    elif byte != 13:
                        line_buffer.append(byte)

            now = time.monotonic()
            if not args.raw and (now - last_render) >= args.refresh:
                render_table(states, parsed_count, bad_count)
                last_render = now

    except KeyboardInterrupt:
        print()
        return 0
    finally:
        os.close(fd)


if __name__ == "__main__":
    raise SystemExit(main())
