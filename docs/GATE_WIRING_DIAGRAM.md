# Simplified Raspberry Pi Gate Wiring

## GPIO behavior

The two switches connect 3.3 V to their GPIO input when active. The program
enables each GPIO's internal pull-down resistor.

- Switch open: input is pulled LOW and is inactive.
- Switch closed to 3.3 V: input reads HIGH and is active.
- Traffic output LOW: red.
- Traffic output HIGH: green.
- Open and close outputs: normally LOW; the selected output goes HIGH for
  exactly one second and then returns LOW.
- Open and close are never driven HIGH together.

## Clean wiring diagram

### Sensor inputs — dry contacts to Raspberry Pi 3.3 V

```text
                  INDUCTIVE-LOOP SWITCH
Physical pin 11 ────────o/ o────────┐
BCM17                                   │
                                        ├──── Physical pin 1 — 3.3 V
Physical pin 13 ────────o/ o────────┘
BCM27             IR-BEAM SWITCH

Switch closed = HIGH / active
Switch open   = LOW / inactive
```

BCM17 and BCM27 use the Raspberry Pi's internal pull-down resistors. Use only
the Pi's 3.3 V rail for these test switches; never connect them to 5 V.

### Outputs — each GPIO requires a protected active-high interface

```text
RASPBERRY PI                 3.3 V ACTIVE-HIGH              GATE EQUIPMENT
40-PIN HEADER                PROTECTED INTERFACE            CONTROL INPUT

Pin 15 / BCM22 ────────────► Channel 1 ───────────────────► Traffic selector
                              LOW = red
                              HIGH = green

Pin 16 / BCM23 ────────────► Channel 2 ───────────────────► OPEN
                              HIGH for 1 second

Pin 18 / BCM24 ────────────► Channel 3 ───────────────────► CLOSE
                              HIGH for 1 second
```

```text
DO NOT CONNECT:

Raspberry Pi GPIO ─────X──── 5 V / 12 V / 24 V gate wiring
```

## Pin table

Use BCM numbers in software. Physical pin numbers refer to the Raspberry Pi
40-pin header.

| Function | BCM GPIO | Physical pin | Electrical behavior |
| --- | ---: | ---: | --- |
| Inductive-loop toggle | 17 | 11 | 3.3 V HIGH means vehicle present |
| IR-beam toggle | 27 | 13 | 3.3 V HIGH means vehicle under barrier |
| Traffic red/green selector | 22 | 15 | LOW red, HIGH green |
| Barrier open command | 23 | 16 | HIGH for one second |
| Barrier close command | 24 | 18 | HIGH for one second |
| Switch supply/common | — | 1 | Raspberry Pi 3.3 V |

Physical pin 17 is another available Raspberry Pi 3.3 V pin.

## Toggle-switch connections

For each input toggle:

```text
BCM GPIO input ---- toggle switch ---- Raspberry Pi 3.3 V
```

Do not connect either input to 5 V, 12 V, or 24 V. A real sensor with a
higher-voltage output must use a 3.3 V-compatible level shifter or opto-isolator.
The internal pull-down provides the inactive LOW state.

## Output requirements

The three outputs are Raspberry Pi 3.3 V logic signals only. They are not
5 V-tolerant and cannot accept or switch 12/24 V directly.

If the traffic controller or barrier inputs are not explicitly compatible with
3.3 V logic, connect each GPIO through an opto-isolator, transistor interface,
or appropriate relay module. The interface must be **active-high** and remain
de-energized when its GPIO input is LOW. Many inexpensive relay boards are
active-low and are unsafe for these default signals without an additional
inverter. Never connect a higher-voltage barrier signal directly to the Pi.

At program startup and shutdown, BCM22, BCM23, and BCM24 are driven LOW. This
selects red and prevents accidental open or close commands.

## Enable automatic gate mode

After wiring and testing the pins, set this in the controller's private `.env`:

```text
GATE_MODE=1
```

The remaining optional values and default pin assignments are listed in
`config/gate.env.example`.
