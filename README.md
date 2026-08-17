# ME306 Project 1 — Integrated X-Y Plotter Firmware

This folder contains the normalized PlatformIO project assembled from the
latest uploaded headers, implementation files, G-code integration program, and
the tested high-power hardware bring-up configuration.

## Application structure

- `src/main.cpp` is intentionally minimal: it forwards `setup()`, `loop()`, and
  the two encoder pin-change interrupt vectors to `PlotterApplication`.
- `app/PlotterApplication.h` and `src/app/PlotterApplication.cpp` own hardware
  construction, update ordering, serial command handling, status reporting,
  telemetry, and startup wiring.
- `system/LimitSafetyManager.h` and
  `src/system/LimitSafetyManager.cpp` encapsulate normal-operation switch
  polling, boundary attribution, M999 reset gating, and temporary expected
  switch release handling.
- `GCodeController` remains responsible for parsing/dispatch and rejects a
  G01 that moves toward a temporarily expected pressed limit.

## Build target

```text
Board: Arduino Mega 2560
Environment: megaatmega2560
Serial monitor: 115200 baud
```

Build with:

```bash
pio run -e megaatmega2560
```

## Serial commands

- `G28` — restart the required homing sequence. One G28 is requested
  automatically at startup.
- `G01 X... Y... F...` — Cartesian linear move in millimetres, with `F` in
  millimetres per minute. `GCODE_POSITIONING_MODE` in `SystemConfig.h` selects
  whether X/Y are absolute machine coordinates or relative offsets.
- `M999` — clear a latched fault. For an `UNEXPECTED_LIMIT` fault, a currently
  pressed switch is recoverable only when the recorded machine position is
  consistent with that switch's configured soft-limit boundary. Any other
  switch still pressed blocks reset and is named in the rejection message.
- `STATUS` — print state, switch states, encoder counts, and machine position.
- `!` — immediate emergency stop without waiting for a newline.
- `STOP`, `X`, or `M112` — emergency stop after the newline is received.

Ordinary status messages begin with `#`. During motion, lines without `#` are
CSV telemetry using the header printed at startup.

## Integrated hardware conventions

- All four limit inputs are active-high and use the junction board's external
  pull-down resistors: `HIGH = pressed`, `LOW = released`.
- Encoder A uses D68/A14 (PCINT22); encoder B uses D52 (PCINT1).
- The verified Cartesian/motor mapping is `A = X - Y`, `B = -X - Y` before
  applying the configured motor count signs.
- Motor output and controller output currently use the full `-255...255`
  range inherited from the successful high-power bring-up program.
- Initial proportional gain is `Kp = 5.0` for both motors; `Ki` and velocity
  feedforward are zero. A and B gains remain separate constants in
  `src/app/PlotterApplication.cpp` for later measured tuning.

## Safety lifecycle

1. Startup automatically requests G28 after hardware and switch
   initialisation. The FSM remains in `HOMING` during the non-blocking routine
   and enters `IDLE` only after both origins are confirmed. A homing failure
   enters `FAULT` through the existing safety path.
2. Homing goes directly to X origin and then Y origin. Each origin uses coarse
   contact, backoff, fine contact, final release, and a saved release snapshot.
3. `MACHINE_X_TRAVEL_MM` and `MACHINE_Y_TRAVEL_MM` in `SystemConfig.h` are the
   measured min-to-max travel values used as fixed soft-limit maxima. They are
   intentionally `0.0f` placeholders in this package; set both to positive
   measured values before G01 can be accepted.
4. Outside HOMING, all four debounced switch states are checked every
   `LIMIT_SAFETY_CHECK_INTERVAL_MS` (currently 10 ms). An unexpected pressed
   limit enters `FAULT` and stops both motors.
5. FAULT and M999 preserve the existing machine-zero flag and loaded soft
   limits.
6. At `UNEXPECTED_LIMIT` entry, each unexpected pressed switch is compared with
   the current Cartesian position and corresponding soft limit using
   `LIMIT_BOUNDARY_TOLERANCE_MM` from `SystemConfig.h` (currently 1.0 mm).
   LEFT/RIGHT map to X min/max and BOTTOM/TOP map to Y min/max.
7. `M999` is rejected while any still-pressed switch was not classified as a
   recoverable boundary switch. Only recoverable switches still pressed at the
   accepted reset become temporarily expected.
8. While a recovery switch remains expected, a `G01` toward that same physical
   limit is rejected before motion is requested. Motion away from it is
   allowed; once its debounced release is observed during movement, the
   exemption is removed and ordinary protection resumes.

## G01 positioning configuration

`SystemConfig.h` contains one compile-time selection:

```cpp
constexpr plotter::GCodePositioningMode GCODE_POSITIONING_MODE = plotter::GCodePositioningMode::ABSOLUTE;
```

- `ABSOLUTE`: at current `(10, 10)`, `G01 X20 Y30` finishes at `(20, 30)`.
- `RELATIVE`: at current `(10, 10)`, `G01 X20 Y30` finishes at `(30, 40)`.

In absolute mode an omitted axis retains its current coordinate. For example,
`G01 X20` changes X without moving Y. The parser normalises both modes to a
per-move displacement, so trajectory generation, limit-recovery direction
checks, coordinated control, and PID code are shared unchanged.

The final power and PID values should still be confirmed on the assembled
machine before the demonstration.
