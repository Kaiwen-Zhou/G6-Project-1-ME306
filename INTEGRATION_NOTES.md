# Integration Notes

## Source selection

- `GCode_Integration_Update1_main.cpp` was used as the final application
  structure because it connects Serial G-code, homing, the FSM, trajectory
  generation, coordinated control, and telemetry.
- The full-output controller configuration from `main(5).cpp` was carried into
  `PlotterApplication`: motor/controller range `-255...255`, initial
  `Kp = 5.0`, `Ki = 0`, and velocity feedforward `0`.
- Upload-generated filename suffixes such as `(2)`, `(3)`, and `(5)` were
  removed. All headers and sources now use their real include paths.
- Reduced `src/main.cpp` to the Arduino framework entry points and encoder ISR
  forwarding. Hardware composition, serial/telemetry flow, and runtime update
  ordering are now owned by `PlotterApplication`.
- Extracted the limit recovery logic into `LimitSafetyManager`, keeping switch
  masks, boundary attribution, M999 gating, low-rate polling, and expected
  release transitions out of both `main.cpp` and the G-code parser.

## Corrections made during integration

- Corrected stale active-low comments in `LimitSwitch.h/.cpp`; the actual code
  and junction-board wiring are active-high.
- Corrected the Converter explanation to match the implemented and tested
  transform: `A = X - Y`, `B = -X - Y`.
- FAULT deliberately preserves `PlotterFSM::machineZeroKnown_` and the G-code
  controller's loaded soft limits, so M999 can resume without G28.
- Added position-based limit-fault attribution in the application layer. At an
  `UNEXPECTED_LIMIT` fault, each unexpected pressed switch is recoverable only
  if the current Cartesian position is within
  `LIMIT_BOUNDARY_TOLERANCE_MM` of its loaded soft-limit boundary. The tolerance
  and the low-rate safety polling interval are both editable in
  `SystemConfig.h`.
- M999 now rejects reset if any currently pressed switch was not recorded as a
  recoverable boundary switch and reports the blocking switch names. Only
  recoverable switches still pressed after an accepted reset become temporarily
  expected.
- Added a pre-motion G01 direction guard for temporarily expected switches:
  commands toward the pressed boundary are rejected, while a move away is
  allowed. The expected bit is removed after the switch's debounced release is
  observed during movement, restoring ordinary limit protection.
- Changed homing from `X max -> X origin -> Y max -> Y origin` to direct
  `X origin -> Y origin`, while preserving coarse/fine contact confirmation,
  final release, and origin count snapshots.
- Replaced homing-measured workspace travel with fixed min-to-max travel values
  in `SystemConfig.h`. Both placeholders are zero so G01 remains safely disabled
  until the measured values are entered.
- Added `GCODE_POSITIONING_MODE` in `SystemConfig.h`. It can compile G01 as
  absolute machine coordinates or relative offsets. The parser converts either
  mode to the existing per-move displacement interface, including correct
  omitted-axis and soft-limit handling.
- Startup now submits G28 automatically after hardware, limit switches, the
  G-code controller, and limit-safety state are initialised. Homing remains
  non-blocking; success reaches IDLE and failure follows the normal FAULT path.
- Expanded homing configuration validation to reject a zero final-release PWM
  or zero final-release timeout.
- Removed obsolete commented field names and parser setter remnants from the
  G-code integration path.
- Updated outdated `PlotterSystem` documentation to describe the integrated
  homing, trajectory, coordinated-control, and fault-stop responsibilities.

## Verification completed

- All source files, including the final `main.cpp`, pass a strict host C++11
  syntax and link check with Arduino/AVR interfaces stubbed.
- Pure-module checks pass for:
  - Cartesian-to-motor-to-Cartesian round trip;
  - fault preserving machine zero through M999/reset;
  - absolute/relative G01 conversion, omitted-axis handling, feedrate limiting,
    and soft-limit rejection;
  - trajectory reaching the exact final reference.
- The integrated host build also covers the position-based M999 gate and the
  pressed-recovery-limit G01 direction guard at compile/link level.

An actual `pio run -e megaatmega2560` should still be performed in the team's
PlatformIO installation because the current assembly environment does not
contain PlatformIO or the AVR compiler.
