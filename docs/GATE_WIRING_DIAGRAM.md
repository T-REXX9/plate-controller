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
- RFID trigger output: normally HIGH; LOW for 1.5 seconds when loop-triggered
  camera capture starts.

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

### Status indicator LEDs

```text
Camera detected       Pin 22 / BCM25 ── 330 Ω ──►|──┐
Server detected       Pin 29 / BCM5  ── 330 Ω ──►|──┤
Loop detector active  Pin 31 / BCM6  ── 330 Ω ──►|──┤
Boom barrier open     Pin 32 / BCM12 ── 330 Ω ──►|──┤── Pin 34 / GND
Plate not recognized  Pin 33 / BCM13 ── 330 Ω ──►|──┘
                                                   LEDs
```

Each GPIO must have its own 220–330 Ω resistor. Connect each resistor to its LED
anode (long leg), then connect every LED cathode (short leg/flat side) to a
Raspberry Pi ground. All indicators are active-high and turn off when the
controller stops.

Indicator meanings:

- **Camera detected:** the camera opened, configured, and supplied a frame.
- **Server detected:** the local Plate Program responded successfully.
- **Loop detector active:** the grounded inductive-loop input reports a vehicle.
- **Boom barrier open:** software has issued OPEN and has not begun CLOSE. This
  is an estimate because the design has no physical barrier-position sensor.
- **Plate not recognized:** the plate was unreadable or the server denied it.
  A network/server failure turns off Server detected instead.

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

Pin 36 / BCM16 ────────────► Channel 4 ───────────────────► RFID capture trigger
                              Idle HIGH; LOW for 1.5 seconds
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
| RFID capture trigger | 16 | 36 | Idle HIGH; LOW for 1.5 seconds at capture start |
| Camera detected LED | 25 | 22 | LOW off; HIGH detected |
| Server detected LED | 5 | 29 | LOW off; HIGH reachable |
| Loop detector active LED | 6 | 31 | LOW off; HIGH vehicle present |
| Boom barrier open LED | 12 | 32 | LOW closed/closing; HIGH open estimate |
| Plate not recognized LED | 13 | 33 | LOW off; HIGH unreadable/denied |
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

If the traffic controller, RFID system, or barrier inputs are not explicitly
compatible with 3.3 V logic, connect each GPIO through an opto-isolator,
transistor interface, or appropriate relay module. The OPEN and CLOSE
interfaces must respond to a LOW GPIO pulse and remain inactive while the GPIO
is HIGH. Never connect a higher-voltage barrier or RFID signal directly to the
Pi.

For boot-time safety, add separate 10 kΩ pull-ups from BCM23, BCM24, and BCM16
to 3.3 V at their protected interfaces. This holds OPEN, CLOSE, and the RFID
trigger inactive before the controller program claims the GPIO lines. Do not
use physical header pins 8 or 10 for the RFID output.

At program startup and shutdown, BCM22 is driven LOW while BCM23, BCM24, and
BCM16 are driven HIGH. This selects red and prevents accidental barrier or RFID
trigger commands.

## Enable automatic gate mode

After wiring and testing the pins, set this in the controller's private `.env`:

```text
GATE_MODE=1
```

The remaining optional values and default pin assignments are listed in
`config/gate.env.example`.
