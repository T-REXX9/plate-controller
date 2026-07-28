# Simplified Raspberry Pi Gate Wiring

## GPIO behavior

The two switches connect their GPIO input to Raspberry Pi ground when active.
The program enables each GPIO's internal pull-up resistor.

- Switch open: input is pulled HIGH and is inactive.
- Switch closed to ground: input reads LOW and is active.
- Traffic output LOW: red.
- Traffic output HIGH: green.
- Open and close outputs: normally HIGH; the selected output goes LOW for
  exactly one second and then returns HIGH.
- Open and close are never driven LOW together.

## Clean wiring diagram

### Sensor inputs — dry contacts to Raspberry Pi ground

```text
                  INDUCTIVE-LOOP SWITCH
Physical pin 11 ────────o/ o────────┐
BCM17                               │
                                    ├──── Physical pin 6 — GND
Physical pin 13 ────────o/ o────────┘
BCM27             IR-BEAM SWITCH

Switch closed = LOW / active
Switch open   = HIGH / inactive
```

BCM17 and BCM27 use the Raspberry Pi's internal pull-up resistors.

### Camera-recognized LED

```text
Physical pin 22 / BCM25 ── 330 Ω resistor ──►|── Physical pin 20 / GND
                                              LED
```

Connect BCM25 through a 220–330 Ω resistor to the LED anode (long leg), then
connect the LED cathode (short leg/flat side) to ground. The LED is off during
startup. It turns on after the controller successfully opens, configures, and
reads a frame from the camera. It turns off if frame capture fails or the
controller stops.

### Outputs — barrier channels require a protected active-low interface

```text
RASPBERRY PI                 PROTECTED INTERFACE            GATE EQUIPMENT
40-PIN HEADER                                               CONTROL INPUT

Pin 15 / BCM22 ────────────► Channel 1 ───────────────────► Traffic selector
                              LOW = red
                              HIGH = green

Pin 16 / BCM23 ────────────► Channel 2 ───────────────────► OPEN
                              Idle HIGH; LOW for 1 second

Pin 18 / BCM24 ────────────► Channel 3 ───────────────────► CLOSE
                              Idle HIGH; LOW for 1 second
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
| Inductive-loop toggle | 17 | 11 | Grounded LOW means vehicle present |
| IR-beam toggle | 27 | 13 | Grounded LOW means vehicle under barrier |
| Traffic red/green selector | 22 | 15 | LOW red, HIGH green |
| Barrier open command | 23 | 16 | Idle HIGH; LOW for one second |
| Barrier close command | 24 | 18 | Idle HIGH; LOW for one second |
| Camera-recognized LED | 25 | 22 | LOW off; HIGH recognized |
| Switch return/common | — | 6 | Raspberry Pi GND |

Other Raspberry Pi ground pins may also be used.

## Toggle-switch connections

For each input toggle:

```text
BCM GPIO input ---- toggle switch ---- Raspberry Pi GND
```

Do not connect either input to 3.3 V, 5 V, 12 V, or 24 V. The internal pull-up
provides the inactive HIGH state. A real powered sensor must use a compatible
dry contact, open-drain interface, or opto-isolator.

## Output requirements

The GPIO outputs are Raspberry Pi 3.3 V logic signals only. They are not
5 V-tolerant and cannot accept or switch 12/24 V directly.

If the traffic controller or barrier inputs are not explicitly compatible with
3.3 V logic, connect each GPIO through an opto-isolator, transistor interface,
or appropriate relay module. The OPEN and CLOSE interfaces must respond to a
LOW GPIO pulse and remain inactive while the GPIO is HIGH. Never connect a
higher-voltage barrier signal directly to the Pi.

For boot-time safety, add a 10 kΩ pull-up from BCM23 to 3.3 V and another from
BCM24 to 3.3 V at the protected interface. This holds both commands inactive
before the controller program claims the GPIO lines.

At program startup and shutdown, BCM22 is driven LOW while BCM23 and BCM24 are
driven HIGH. This selects red and prevents accidental open or close commands.

## Enable automatic gate mode

After wiring and testing the pins, set this in the controller's private `.env`:

```text
GATE_MODE=1
```

The remaining optional values and default pin assignments are listed in
`config/gate.env.example`.
