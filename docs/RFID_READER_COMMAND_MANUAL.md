# UHF RFID Reader Command Manual

## Purpose

This manual records the serial commands confirmed during testing of the UHF RFID reader. The intended configuration is:

- RS232 serial communication at 9600 baud
- Answer Mode (no automatic or continuous tag broadcasting)
- The Raspberry Pi requests a tag read only when needed
- Commands and responses are transmitted as raw hexadecimal bytes

## Serial terminal settings

Use these settings in CuteCom, QtCom, or another serial terminal:

| Setting | Value |
|---|---|
| Baud rate | 9600 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None |
| Input format | Hexadecimal/raw bytes |

Do not send the commands as ordinary text. Do not append a newline, carriage return, or spaces as transmitted characters. Spaces in this manual only separate the hexadecimal bytes for readability.

## Recommended configuration sequence

### 1. Request reader information

Send:

```text
04 00 21 D9 6A
```

Confirmed response:

```text
0D 00 21 00 05 02 86 02 31 80 1A 14 97 5C
```

Important values from this response:

- Status: `00` — command succeeded
- Firmware version: `05 02`
- Reader type: `86`
- Tag protocol: `02` — EPC Gen2
- RF power: `1A` — 26 dBm
- Inventory scan time: `14` — 20 × 100 ms, or 2 seconds

### 2. Change the baud rate to 9600

Send:

```text
05 FF 28 00 76 46
```

After sending this command, close the serial connection, change the terminal baud rate to 9600, and reconnect. Request the reader information again to confirm communication.

### 3. Set the inventory scan duration to two seconds

Send:

```text
05 00 25 14 58 66
```

Confirmed successful response:

```text
05 00 25 00 FD 30
```

The `14` data byte is hexadecimal 20. The reader uses units of 100 ms, so the configured duration is 2 seconds.

To set the duration to five seconds instead, send:

```text
05 00 25 32 6C 22
```

The `32` data byte is hexadecimal 50, or 50 × 100 ms.

### 4. Read the current work-mode configuration

Send:

```text
04 00 36 E7 0E
```

Response observed before Answer Mode was enabled:

```text
11 00 36 00 00 1E 0A 0F 01 02 04 02 06 00 00 00 A6 66
```

The response layout is:

```text
LEN ADR CMD STATUS WG_MODE WG_INTERVAL WG_WIDTH WG_PULSE READ_MODE MODE_STATE MEM_INVEN FIRST_ADR WORD_NUM TAG_TIME ACCURACY OFFSET CRC_L CRC_H
```

In the observed response, `READ_MODE` was `01`, meaning Scan Mode. This was why the reader continuously transmitted `EE` tag packets.

### 5. Set Answer Mode and stop automatic broadcasting

Send:

```text
0A 00 35 00 02 04 02 06 00 CD 09
```

Confirmed successful response:

```text
05 00 35 00 6C A5
```

This command preserves the reader's other six-byte work-mode configuration while changing `READ_MODE` from `01` (Scan Mode) to `00` (Answer Mode).

The setting is stored by the reader. Continuous tag broadcasting should remain disabled after a power cycle.

### 6. Verify Answer Mode

Send the work-mode query again:

```text
04 00 36 E7 0E
```

In the response, count past the four Wiegand fields. The `READ_MODE` byte should now be:

```text
00
```

## Reading an RFID tag on demand

### Single-tag inventory

Send:

```text
04 00 0F A5 A2
```

In Answer Mode, this command performs one single-tag inventory operation. Place only one tag in the reader's field for the most reliable result.

The response command should be `0F`. For this firmware, a successful response is:

```text
LEN ADR 0F 01 NUM EPC_LENGTH EPC_BYTES... CRC_L CRC_H
```

For example, the confirmed frame below contains the 12-byte EPC
`E284361100001000094944AA`; `DA FF` is the CRC and is not part of the tag:

```text
13 00 0F 01 01 0C E2 84 36 11 00 00 10 00 09 49 44 AA DA FF
```

If the reader sends repeated packets with response command `EE`, it is still operating in Scan Mode. Repeat the Answer Mode command from step 5 and verify the mode with command `36`.

### Multi-tag inventory for this firmware

Send:

```text
04 00 01 DB 4B
```

This uses the configured inventory duration and returns the EPC values detected
during that window. Do not send `06 00 01 02 00 7C 62` to this reader: its older
firmware interprets the two data bytes differently from newer protocol versions.

## Status and error bytes

| Status | Meaning |
|---|---|
| `00` | Success |
| `FD` | Command length is incorrect |
| `FE` | Command is unsupported or illegal in the current mode |

## Commands that did not solve the continuous broadcast

Do not use these as the Answer Mode configuration for this reader:

```text
04 00 93 40 FC
```

Command `93` did not stop the automatic `EE` broadcasts. It is intended to stop an executing command-`01` inventory, not to change a persistent Scan Mode setting.

```text
05 00 76 00 62 C9
```

Command `76` returned `FE` and is not the correct work-mode command for this reader firmware.

```text
05 00 35 00 6C A5
```

This short form of command `35` returned `FD` because this reader requires all six work-mode parameter bytes. Use the complete `0A 00 35 ...` command documented above.

## Normal Raspberry Pi operating flow

1. Open the RS232 serial port at 9600 baud, 8-N-1, with no flow control.
2. Keep the reader in Answer Mode while the gate controller is idle.
3. When the loop detector signals that a vehicle is present, send the single-tag inventory command.
4. Read and validate one complete response frame.
5. Extract and normalize the EPC/tag value.
6. Send the value to the Plate Program server for database authorization.
7. Ignore duplicate reads from the same gate cycle.

## Quick command reference

| Operation | Command |
|---|---|
| Get reader information | `04 00 21 D9 6A` |
| Set baud rate to 9600 | `05 FF 28 00 76 46` |
| Set scan duration to 2 seconds | `05 00 25 14 58 66` |
| Set scan duration to 5 seconds | `05 00 25 32 6C 22` |
| Get work-mode configuration | `04 00 36 E7 0E` |
| Set Answer Mode | `0A 00 35 00 02 04 02 06 00 CD 09` |
| Perform one single-tag inventory | `04 00 0F A5 A2` |
| Perform multi-tag inventory | `04 00 01 DB 4B` |
