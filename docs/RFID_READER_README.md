# UHF RFID Reader Communication README

This file records the serial settings, commands, and response formats that were
actually observed while testing the long-range UHF RFID reader. It is intended
to prevent future implementations from repeating the command and response
parsing problems encountered during testing.

## Required serial connection

Use a hardware UART whenever possible.

### Raspberry Pi UART wiring

The RFID reader uses RS232 voltage levels. Place a proper RS232-to-TTL level
converter, such as a MAX3232 module, between the reader and Raspberry Pi.

| Raspberry Pi | Physical pin | Converter TTL side |
|---|---:|---|
| GPIO14 / TXD | 8 | RXD |
| GPIO15 / RXD | 10 | TXD |
| Ground | 6 | GND |

Transmit and receive must cross: Pi TX goes to converter RX, and converter TX
goes to Pi RX. Never connect true RS232 signals directly to Raspberry Pi GPIO.

The usual Linux device is:

```text
/dev/serial0
```

Disable the Linux serial login console before using this port for the reader.

## Serial settings

| Setting | Value |
|---|---|
| Baud rate | 9600 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None |

Commands are binary bytes. The spaces shown in this README are only for human
readability. Do not transmit the characters `0`, `6`, spaces, or a newline.
For example, the inventory command is seven raw bytes beginning with `0x06`.

## Primary inventory command

The command confirmed for starting an RFID inventory is:

```text
06 00 01 02 00 7C 62
```

This reader does not necessarily return the tag immediately. Keep the serial
port open and continue reading for at least the configured inventory duration.
A 10- to 12-second receive deadline is recommended when the reader is configured
for a five-second inventory period.

Do not clear, flush, close, or reconfigure the receive port after sending the
command. Doing so can discard the delayed response.

### Inventory acknowledgement

The following response was observed after sending the command:

```text
07 00 01 01 00 09 07 CF
```

This command-`01` frame acknowledges that the inventory operation was accepted.
It is not the RFID EPC. Continue listening after receiving it.

### RFID tag broadcast

The actual tag arrives later in a command-`EE` frame. A confirmed example is:

```text
11 00 EE 00 E2 84 36 11 00 00 10 00 09 49 44 AA 9A CC
```

Its layout is:

```text
LEN ADR EE STATUS EPC_BYTES... CRC_LOW CRC_HIGH
```

For that frame:

- `11` is the frame length field; the complete frame contains `0x11 + 1`, or
  18 bytes.
- `00` is the reader address.
- `EE` identifies a tag broadcast.
- The next `00` is the status/metadata byte.
- `E2 84 36 11 00 00 10 00 09 49 44 AA` is the 12-byte EPC.
- `9A CC` is the CRC and is not part of the EPC.

The normalized RFID value is therefore:

```text
E284361100001000094944AA
```

For an `EE` frame, the EPC starts at byte offset 4. Its length is:

```text
complete_frame_length - 6
```

The subtraction removes `LEN`, `ADR`, `CMD`, `STATUS`, and the two CRC bytes.

The reader may send the same `EE` frame repeatedly during one inventory window.
Use the first valid EPC needed for the current gate cycle and ignore duplicates.

## Single-tag Answer Mode command

The following command also produced direct single-tag responses during testing:

```text
04 00 0F A5 A2
```

It is useful when the reader is correctly configured for Answer Mode. A
successful direct response has this layout:

```text
LEN ADR 0F 01 TAG_COUNT EPC_LENGTH EPC_BYTES... CRC_LOW CRC_HIGH
```

Confirmed example:

```text
13 00 0F 01 01 0C E2 84 36 11 00 00 10 00 09 49 44 AA DA FF
```

The EPC length is `0C`, or 12 bytes, and the EPC is:

```text
E284361100001000094944AA
```

A confirmed no-tag response is:

```text
05 00 0F FB E2 A7
```

For the current tested workflow, use `06 00 01 02 00 7C 62` and capture the
delayed `EE` response. Keep `04 00 0F A5 A2` as a direct Answer Mode alternative.

## Reader information

Send:

```text
04 00 21 D9 6A
```

Confirmed response:

```text
0D 00 21 00 05 02 86 02 31 80 1A 14 97 5C
```

Observed values include:

- Firmware version: `05 02`
- Reader type: `86`
- Tag protocol: `02` (EPC Gen2)
- RF power: `1A` (26 dBm)
- Inventory duration: `14` hexadecimal = 20 units = 2 seconds

## Configure inventory duration

### Two seconds

Send:

```text
05 00 25 14 58 66
```

Confirmed success response:

```text
05 00 25 00 FD 30
```

### Five seconds

Send:

```text
05 00 25 32 6C 22
```

The duration byte uses 100 ms units. `32` hexadecimal is 50, and 50 × 100 ms
equals five seconds.

The information response below was observed after setting five seconds; the
duration byte is `32`:

```text
0D 00 21 00 05 02 86 02 31 80 1A 32 A3 18
```

## Read work-mode configuration

Send:

```text
04 00 36 E7 0E
```

Confirmed response:

```text
11 00 36 00 00 1E 0A 0F 01 02 04 02 06 00 00 00 A6 66
```

The observed response showed `READ_MODE = 01`, which explained continuous tag
broadcasting.

## Configure Answer Mode

The complete command that successfully stopped automatic broadcasting was:

```text
0A 00 35 00 02 04 02 06 00 CD 09
```

Confirmed success response:

```text
05 00 35 00 6C A5
```

This is a complete work-mode configuration frame. Do not replace it with the
short command `05 00 35 00 6C A5`; that short byte sequence is the response,
not the configuration command.

## Set the reader baud rate to 9600

Send:

```text
05 FF 28 00 76 46
```

After sending it, close the port, reconnect at 9600 baud, and send the reader
information command to confirm communication.

## Packet framing and CRC

The first byte is the protocol length field. The complete number of bytes in a
frame is:

```text
frame[0] + 1
```

The last two bytes are CRC-16 in low-byte, high-byte order. The CRC starts at
`0xFFFF` and uses polynomial `0x8408`, processing every byte except the two CRC
bytes.

Only accept a packet after all of these checks pass:

1. At least six bytes are available.
2. The complete frame length equals `frame[0] + 1`.
3. The calculated CRC matches the final two bytes.
4. The command byte identifies the expected response type.
5. The EPC boundaries fit completely inside the frame.

Serial reads can split one response across multiple operating-system reads or
combine multiple frames into one read. Accumulate bytes in a buffer and parse
complete length-delimited frames instead of assuming one `read()` equals one
packet.

## Reliable inventory procedure

1. Open `/dev/serial0` at 9600, 8-N-1, with no flow control.
2. Discard stale bytes before starting a new gate cycle.
3. Send the seven raw bytes `06 00 01 02 00 7C 62` exactly once.
4. Keep the same port open and continuously poll/read it.
5. Accept the command-`01` frame as an acknowledgement only.
6. Continue waiting for a CRC-valid command-`EE` frame.
7. Extract the EPC from offset 4 through the byte before the CRC.
8. Normalize the EPC as uppercase hexadecimal without spaces or hyphens.
9. Ignore repeated copies after accepting the first required EPC.
10. Report a timeout only after the full 10- to 12-second receive window.

## Troubleshooting

### Command transmits but RX is empty

- Confirm reader TX is connected to Pi RX, physical pin 10.
- Confirm reader RX is connected to Pi TX, physical pin 8.
- Connect converter and Pi grounds.
- Confirm the converter is RS232-to-3.3-V-TTL compatible.
- Confirm 9600 baud, 8-N-1, and no flow control.
- Confirm the Linux serial console is disabled on `/dev/serial0`.
- Do not close or flush the receive port immediately after transmitting.
- Wait longer than the configured inventory duration.

### An acknowledgement arrives but no EPC arrives

- The reader accepted the command, so TX and at least part of RX are working.
- Place a tag inside the antenna field before sending the command.
- Wait for a delayed `EE` broadcast rather than treating the `01` acknowledgement
  as the EPC.
- Check antenna connection and RF power.

### Tags broadcast continuously

- Send the complete Answer Mode configuration command.
- Verify the work mode again with command `04 00 36 E7 0E`.
- During a commanded inventory window, repeated `EE` packets are expected; stop
  processing duplicates after obtaining the required EPC.

## Quick reference

| Operation | Raw hexadecimal command |
|---|---|
| Start tested inventory cycle | `06 00 01 02 00 7C 62` |
| Direct single-tag Answer Mode inventory | `04 00 0F A5 A2` |
| Read reader information | `04 00 21 D9 6A` |
| Set inventory duration to 2 seconds | `05 00 25 14 58 66` |
| Set inventory duration to 5 seconds | `05 00 25 32 6C 22` |
| Read work-mode configuration | `04 00 36 E7 0E` |
| Configure Answer Mode | `0A 00 35 00 02 04 02 06 00 CD 09` |
| Set baud rate to 9600 | `05 FF 28 00 76 46` |
