# VEX V5 PID Motion-Control Experiments

A Python control-development project for VEX V5 drive, turn, and swing motion. The central `PIDController` class implements proportional control, gated integral accumulation, and derivative-on-measurement feedback. Robot-specific helpers translate its output into motor voltages.

**Status:** development snapshot, not a drop-in robot program. The current file mixes a two-motor active configuration with six-motor helpers; reconcile the configuration before running it on hardware.

## Implementation

The implementation in [`PID_controller.py`](PID_controller.py) computes:

```text
error[k]      = target[k] - measurement[k]
integral[k]   = clamped sum of eligible errors
output[k]     = kP * error[k] + kI * integral[k]
                - kD * (measurement[k] - measurement[k-1])
```

Integral accumulation is enabled only when `abs(error) < start_i`, is limited to ±100 in the controller, and can reset on an error-sign change. The default `start_i=0.0` disables accumulation; a nonzero `kI` alone does not enable integral action.

The derivative acts on the measurement rather than the error. Sample time is not an explicit argument: the gains are tied to the effective loop interval. Individual motion helpers use requested waits of 10 or 20 milliseconds, in addition to processing time. These delays are not measured real-time guarantees.

## Control paths

```text
Distance / angle target
          |
          v
PIDController.calculate() <── encoder or inertial measurement
          |
          v
Motion helper / output limits ──> motor voltage ──> drivetrain
```

The file includes straight-drive, relative-turn, absolute-heading, swing-turn, and combined distance/heading helpers. Termination uses tolerance and timeout checks, with a settling interval in the combined helper.

## Using the source

Use a VEXcode Python environment for V5 hardware. Desktop Python cannot directly run the device configuration or the `vex` import. Review motor ports, reversal flags, wheel diameter, gear ratio, sensor ports, and controller gains against the intended robot before downloading.

The integrated competition implementation is maintained separately in [197B High Stakes](https://github.com/noteric3a/197Bhighstakes); this repository retains the standalone development snapshot.

## Known integration work

- Several helpers reference back/middle drive motors and `drivetrain`, while those definitions are currently inside a commented six-motor example.
- The combined distance/heading helper calculates a normalized heading error for settling, but passes raw headings to its PID calculation. Wraparound behavior needs explicit tests.
- Its linear and angular outputs are limited individually before summation. Review the final per-motor voltage limit as part of hardware integration.
- Controller state persists between calls. Define and test the reset behavior needed between maneuvers.

Documentation does not resolve these implementation issues or establish safe physical behavior. Start with wheels raised and verify disable/timeout behavior before autonomous motion.

## Validation to record

No measured overshoot, settling-time, or accuracy results are published here. A reproducible control report should include target/actual response plots, gain values, loop timing, battery condition, surface/load, repeated trials, and tests around ±180°/0° heading boundaries. Separate hardware measurements from simulations.

## Attribution

The source includes VEXcode-generated configuration and project-specific control logic. Preserve generated-code labels and credit team or external algorithm contributions when confirmed.
