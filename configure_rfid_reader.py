#!/usr/bin/env python3
"""Configure a UHFReader18-compatible reader for controller operation."""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time


READER_INFO = bytes.fromhex("04 00 21 D9 6A")
SET_BAUD_9600 = bytes.fromhex("05 FF 28 00 76 46")
SET_ANSWER_MODE = bytes.fromhex("0A 00 35 00 02 04 02 06 00 CD 09")
GET_WORK_MODE = bytes.fromhex("04 00 36 E7 0E")

BAUD_CONSTANTS = {
    1200: termios.B1200,
    2400: termios.B2400,
    4800: termios.B4800,
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
}


def frame_crc(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8408) if (crc & 1) else (crc >> 1)
    return crc


def valid_frame(frame: bytes) -> bool:
    if len(frame) < 6 or len(frame) != frame[0] + 1:
        return False
    received = frame[-2] | (frame[-1] << 8)
    return received == frame_crc(frame[:-2])


def extract_frames(buffer: bytearray) -> list[bytes]:
    frames: list[bytes] = []
    while len(buffer) >= 6:
        match: tuple[int, int, bytes] | None = None
        for offset in range(len(buffer) - 5):
            total = buffer[offset] + 1
            if total < 6 or total > 129 or offset + total > len(buffer):
                continue
            candidate = bytes(buffer[offset : offset + total])
            if valid_frame(candidate):
                match = (offset, total, candidate)
                break
        if match is None:
            # Preserve enough trailing data for a response split across reads,
            # but prevent unlimited growth when the line contains noise.
            if len(buffer) > 258:
                del buffer[:-129]
            break
        offset, total, candidate = match
        frames.append(candidate)
        del buffer[: offset + total]
    return frames


def open_uart(device: str, baud: int) -> int:
    descriptor = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    settings = termios.tcgetattr(descriptor)
    settings[0] = 0
    settings[1] = 0
    settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    settings[3] = 0
    settings[4] = BAUD_CONSTANTS[baud]
    settings[5] = BAUD_CONSTANTS[baud]
    settings[6][termios.VMIN] = 0
    settings[6][termios.VTIME] = 0
    termios.tcsetattr(descriptor, termios.TCSANOW, settings)
    termios.tcflush(descriptor, termios.TCIOFLUSH)
    return descriptor


def write_all(descriptor: int, command: bytes) -> None:
    offset = 0
    while offset < len(command):
        try:
            written = os.write(descriptor, command[offset:])
        except BlockingIOError:
            select.select([], [descriptor], [], 0.5)
            continue
        if written <= 0:
            raise RuntimeError("RFID serial write returned no data")
        offset += written
    termios.tcdrain(descriptor)


def transact(
    descriptor: int,
    command: bytes,
    expected_command: int,
    timeout: float = 3.0,
) -> bytes:
    # Clear stale bytes before, never after, transmitting the command.
    termios.tcflush(descriptor, termios.TCIFLUSH)
    write_all(descriptor, command)
    deadline = time.monotonic() + timeout
    received = bytearray()

    while time.monotonic() < deadline:
        remaining = max(0.01, deadline - time.monotonic())
        readable, _, _ = select.select([descriptor], [], [], remaining)
        if not readable:
            continue
        try:
            chunk = os.read(descriptor, 256)
        except BlockingIOError:
            continue
        if not chunk:
            continue
        received.extend(chunk)
        for frame in extract_frames(received):
            if frame[2] == expected_command:
                return frame

    suffix = f" Received: {received.hex(' ').upper()}" if received else ""
    raise TimeoutError(
        f"Reader did not return command 0x{expected_command:02X}." + suffix
    )


def show(label: str, frame: bytes) -> None:
    print(f"[ OK ] {label}: {frame.hex(' ').upper()}")


def configure_reader(device: str, current_baud: int) -> None:
    if not os.path.exists(device):
        raise RuntimeError(f"Serial device does not exist: {device}")

    print(f"[RFID] Opening {device} at {current_baud} baud...")
    descriptor = open_uart(device, current_baud)
    try:
        info = transact(descriptor, READER_INFO, 0x21)
        if info[3] != 0x00:
            raise RuntimeError(f"Reader information failed with status 0x{info[3]:02X}")
        show("Reader communication confirmed", info)

        if current_baud != 9600:
            print("[RFID] Changing reader baud rate to 9600...")
            try:
                response = transact(descriptor, SET_BAUD_9600, 0x28, timeout=2.0)
                show("Baud command accepted", response)
            except TimeoutError:
                # Some firmware switches baud before its response can be read.
                print("[INFO] No old-baud reply; reconnecting at 9600 for verification.")
    finally:
        os.close(descriptor)

    time.sleep(0.5)
    descriptor = open_uart(device, 9600)
    try:
        info = transact(descriptor, READER_INFO, 0x21)
        if info[3] != 0x00:
            raise RuntimeError(f"9600-baud verification returned status 0x{info[3]:02X}")
        show("9600 baud verified", info)

        print("[RFID] Setting Answer Mode (automatic broadcasting OFF)...")
        answer = transact(descriptor, SET_ANSWER_MODE, 0x35)
        if answer[3] != 0x00:
            raise RuntimeError(f"Answer Mode command returned status 0x{answer[3]:02X}")
        show("Answer Mode command accepted", answer)

        mode = transact(descriptor, GET_WORK_MODE, 0x36)
        if mode[3] != 0x00:
            raise RuntimeError(f"Work-mode query returned status 0x{mode[3]:02X}")
        if len(mode) <= 8:
            raise RuntimeError("Work-mode response was shorter than expected")
        if mode[8] != 0x00:
            raise RuntimeError(
                f"Answer Mode verification failed: READ_MODE is 0x{mode[8]:02X}"
            )
        show("Answer Mode verified", mode)
    finally:
        os.close(descriptor)

    print("[PASS] RFID reader is configured for 9600 baud and Answer Mode.")


def self_test() -> None:
    confirmed = bytes.fromhex(
        "13 00 0F 01 01 0C E2 84 36 11 00 00 10 00 09 49 44 AA DA FF"
    )
    assert valid_frame(confirmed)
    buffer = bytearray(b"noise" + confirmed)
    assert extract_frames(buffer) == [confirmed]
    assert frame_crc(READER_INFO[:-2]).to_bytes(2, "little") == READER_INFO[-2:]
    print("RFID configuration helper self-test passed.")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/serial0")
    parser.add_argument("--current-baud", type=int, choices=sorted(BAUD_CONSTANTS))
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return 0
    if args.current_baud is None:
        parser.error("--current-baud is required unless --self-test is used")

    try:
        configure_reader(args.device, args.current_baud)
    except (OSError, RuntimeError, TimeoutError) as error:
        print(f"[FAIL] {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
