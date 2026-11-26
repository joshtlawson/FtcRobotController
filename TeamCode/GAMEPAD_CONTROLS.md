# TeleOp gamepad controls

These controls apply to the Red and Blue 26/27 TeleOp modes. Button names use
the FTC gamepad labels; R1 is the right bumper and the triggers are analog.

## Gamepad 1 - driver

```text
       LEFT TRIGGER: strafe left        RIGHT TRIGGER: strafe right
     +-------------------------------------------------------------+
     |                                                             |
     |  LEFT STICK                     FACE BUTTONS (hold):        |
     |  up/down: drive forward/back               Y: -2500         |
     |                                     X: -3000  B: -3500      |
     |                  RIGHT STICK               A: -4000         |
     |                  left/right: turn                           |
     |                                                             |
     |  D-PAD (hold): launcher speed                               |
     |            up: -2000                                        |
     |   left: -800       right: -1500                              |
     |            down: -1000                                      |
     +-------------------------------------------------------------+
```

Launcher speeds are motor velocity commands in encoder ticks per second. The
face buttons and D-pad are manual presets: hold one to run the launcher, and
release it to stop. Presets are ignored while gamepad 2 holds R1. If more than
one preset is held, priority is A, B, X, Y, D-pad up, right, down, then left.
If both strafe triggers are pressed, the left trigger takes priority.

## Gamepad 2 - operator

```text
                         R1 / RIGHT BUMPER
                      hold: automatic launch
     +-------------------------------------------------------------+
     |                                                             |
     |  D-PAD: intake                 FACE BUTTONS (press):        |
     |       up: +2000                         Y: auto speed -100  |
     |       down: -2000                 X: auto speed +100        |
     |                                        A: toggle ramp       |
     |                                                             |
     |                       RIGHT STICK                           |
     |                       left/right: manual turret yaw         |
     +-------------------------------------------------------------+
```

- Hold D-pad up or down to run the intake in the indicated motor direction;
  release to stop. Down takes priority if both are held. Manual intake and
  turret controls are unavailable while R1 is held.
- Press A to toggle the flower ramp and dozer between raised and lowered
  positions.
- Press X or Y to adjust the automatic launcher setpoint by 100 ticks/sec.
  The initial setpoint is 2050 ticks/sec, bounded by 100 and 4000; it is
  displayed in telemetry.
- Hold R1 to aim and launch automatically. The launcher spins up and the
  gate opens once the target is detected and aligned. Once launching starts,
  it continues until R1 is released, even if the target is lost. Releasing
  R1 closes the gate and stops the automatic launcher and intake.

## Calibration controls currently enabled

Both `SetDozerPositionRoutine` and `SetThroatPositionRoutine` are initialized
to `true` in `RobotControl.java`. These controls overlap normal controls:

```text
 GAMEPAD 2  X: dozer position +0.01     Y: dozer position -0.01
            A: throat position +0.01    B: throat position -0.01
```

The dozer uses its calibration position when the ramp position is commanded;
the throat uses its calibration position instead of its normal pollen/nectar
positions. A also toggles the ramp, and X/Y are shared with launcher speed.
The routines use the same press-event methods as those normal controls, so
their interaction depends on event consumption order.
