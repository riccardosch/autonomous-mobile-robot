#!/usr/bin/env python3
"""Send wheel speed targets to the wheel firmware at a fixed rate (test T5).

Writes "v <left> <right>" (rad/s) every 1/rate seconds and prints what the
ESP32 sends back. Ctrl+C stops sending; the script then keeps printing for a
few seconds, so you can see the firmware watchdog stop the wheels.

Close `pio device monitor` first: only one program at a time can open the port.

    python3 tools/cmd_sender.py                       # 4 rad/s, 20 Hz, /dev/ttyUSB0
    python3 tools/cmd_sender.py --left 2 --right -2 --rate 10
"""

import argparse
import time

import serial

AFTER_STOP_S = 3.0  # how long to keep printing after Ctrl+C


def print_lines(port, buf):
    """Print the complete lines received so far; return the incomplete rest."""
    buf += port.read(port.in_waiting or 1)
    *lines, rest = buf.split(b"\n")
    for line in lines:
        print(line.decode(errors="replace").rstrip())
    return rest


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--left", type=float, default=4.0, help="left target, rad/s")
    parser.add_argument("--right", type=float, default=4.0, help="right target, rad/s")
    parser.add_argument("--rate", type=float, default=20.0, help="commands per second")
    args = parser.parse_args()

    port = serial.Serial()
    port.port = args.port
    port.baudrate = args.baud
    port.timeout = 0.01
    # Keep DTR and RTS released: on the DevKit they drive the reset and boot pins.
    port.dtr = False
    port.rts = False
    port.open()

    command = f"v {args.left:.3f} {args.right:.3f}\n".encode()
    period = 1.0 / args.rate
    buf = b""
    print(f"sending {command.decode().strip()} at {args.rate:g} Hz, Ctrl+C to stop")
    try:
        next_send = time.monotonic()
        while True:
            if time.monotonic() >= next_send:
                port.write(command)
                next_send += period
            buf = print_lines(port, buf)
    except KeyboardInterrupt:
        stop = time.monotonic()
        print(f"\n--- stopped sending; the watchdog should stop the wheels "
              f"(printing for {AFTER_STOP_S:g} s more)")
        while time.monotonic() - stop < AFTER_STOP_S:
            buf = print_lines(port, buf)
    finally:
        port.close()


if __name__ == "__main__":
    main()
