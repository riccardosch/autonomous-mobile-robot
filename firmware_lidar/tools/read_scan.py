#!/usr/bin/env python3
"""Read the binary scan frames sent by the LiDAR firmware (esp32c3 environment).

Frame layout, little-endian:
    sync 2 (A5 5A) | version 1 | seq 2 | scan_duration_us 4 | bins 2 |
    distances_mm 2*bins | crc16 2 (CCITT-FALSE over everything after sync)

Usage:
    python3 read_scan.py [--port /dev/ttyACM0] [--dump]
"""

import argparse
import binascii
import struct
import time

import serial

SYNC = b"\xA5\x5A"
HEADER = struct.Struct("<BHIH")  # version, seq, scan_duration_us, bins
FRAME_VERSION = 1


def default_crc_error():
    print("CRC error, resyncing")


def read_frames(port, on_crc_error=default_crc_error):
    """Yield (seq, scan_duration_us, distances_mm) for every valid frame."""
    buf = bytearray()
    while True:
        buf += port.read(port.in_waiting or 1)
        while True:
            start = buf.find(SYNC)
            if start < 0:
                del buf[:-1]  # keep a possible first sync byte
                break
            del buf[:start]
            if len(buf) < 2 + HEADER.size:
                break
            version, seq, duration_us, bins = HEADER.unpack_from(buf, 2)
            if version != FRAME_VERSION or bins == 0 or bins > 3600:
                del buf[:1]  # false sync, look for the next one
                continue
            frame_len = 2 + HEADER.size + 2 * bins + 2
            if len(buf) < frame_len:
                break
            body = bytes(buf[2:frame_len - 2])
            (crc,) = struct.unpack_from("<H", buf, frame_len - 2)
            if binascii.crc_hqx(body, 0xFFFF) != crc:
                on_crc_error()
                del buf[:1]
                continue
            distances = struct.unpack_from(f"<{bins}H", body, HEADER.size)
            del buf[:frame_len]
            yield seq, duration_us, distances


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--dump", action="store_true", help="print every distance of each scan")
    args = parser.parse_args()

    with serial.Serial(args.port, 115200, timeout=1) as port:
        last_seq = None
        lost = 0
        t0, count = time.monotonic(), 0
        for seq, duration_us, distances in read_frames(port):
            if last_seq is not None:
                lost += (seq - last_seq - 1) & 0xFFFF
            last_seq = seq
            count += 1

            valid = sum(1 for d in distances if d)
            rate = 1e6 / duration_us if duration_us else 0.0
            print(
                f"seq {seq:5d} | {rate:4.1f} Hz | valid {valid:3d}/{len(distances)} | "
                f"0:{distances[0]} 90:{distances[90 % len(distances)]} "
                f"180:{distances[180 % len(distances)]} 270:{distances[270 % len(distances)]} mm | "
                f"lost frames {lost}"
            )
            if args.dump:
                print(" ".join(str(d) for d in distances))

            elapsed = time.monotonic() - t0
            if elapsed >= 5:
                print(f"--- {count / elapsed:.2f} frames/s received ---")
                t0, count = time.monotonic(), 0


if __name__ == "__main__":
    main()
