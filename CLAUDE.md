# Working in this repository

Notes for anyone — human or agent — picking this up. Not a style guide; these are
the things that have actually gone wrong here, and what to do instead.

## What this is

An FTC (SDK 11.1) template for a mecanum robot with a **goBILDA Pinpoint**
odometry computer and a **Limelight 3A**, fused into one field pose by a WPILib-port
Kalman estimator. On top of that: a Bézier path follower, a shoot-on-the-move
aiming solution for a robot with **no turret** (it pivots the chassis), and an
FRC-shaped command framework.

Read `README.md` for how it works, `MANUAL.md` to run it, and the four docs next to
the code (`LOCALIZATION.md`, `pathing/PATHING.md`, `shooting/SHOOTING.md`,
`lib/command/COMMANDS.md`) for the parts you are touching.

## Verify before you claim anything

```bash
./tools/verify/run.sh        # must exit 0
```

Phase 1 typechecks the whole `TeamCode` tree against hand-written FTC SDK stubs in
`tools/verify/stub/`. Phase 2 runs numeric suites, including a full simulated red
and blue match.

There is **no Android SDK and no Gradle cache in this environment**, so the APK
cannot be built here. `run.sh` is the only check available. A green run means the
logic holds and the code typechecks against that stub subset; it is not a
substitute for building the real app before a competition. Say so rather than
implying the app was built.

**Anything numeric gets a check.** Two bugs here survived a passing test suite
because the tests fed hand-picked inputs instead of going through the real code
path: a vision standard-deviation coefficient carried over from WPILib in metres
made the Kalman gain ≈0.0002, silently disabling fusion, and
`getBotposeAvgDist()`'s metres were read as inches. Both were invisible until a
test computed the gain with the real formula. If you add a constant that feeds a
formula, add a check that exercises the formula.

Adding an SDK stub is fine and normal — keep it minimal and honest about being a
stub.

## Measure, do not reason

The instinct to reason about a control loop and then write the conclusion down is
wrong often enough here to be worth naming.

- A change reserving turn authority "so aiming can always keep up" was written,
  argued for, and measured: 154 loops → 156. It earned nothing and was reverted.
  `HEADING_kP` is the actual lever (0.30 → 156 loops turning, 0.60 → 97, 1.00 → 55),
  and that measurement is now in `PathConstants`.
- A closed-form angular feedforward was 17% wrong because the virtual goal drifts
  while you differentiate. Re-solving the whole aim 1 ms later and differencing
  took the worst miss to 0.0018 in.
- A fixed three-pass virtual-goal iteration missed by 1.6 in. A convergence loop
  does not.

The simulation harness has also been wrong three times (direction double-counting,
field-frame vs robot-frame drift, measuring shot quality over frames the robot had
refused to shoot on). When a result is surprising, suspect the harness too.

## Units and signs

- **Inches and radians throughout team code.** Degrees only at telemetry and in
  `Rotation2d.fromDegrees` at authoring sites.
- **The Limelight is native metres.** `VisionConstants.BOTPOSE_AVG_DIST_UNIT`
  exists because of this; convert, never assume. Do not put a unit in a name
  (`_MM`) unless the value is in it.
- **CCW positive** for every angle and angular rate. Field +X and +Y per
  `README.md`.
- Motor directions are left `REVERSE`, right `FORWARD`. All four the same does not
  reverse a mecanum, it swaps translation and rotation — "drive forward" spins in
  place. `DrivetrainDirectionCheck` exists to catch it.
- `PathConstants.HEADING_CORRECTION_SIGN` is `+1.0`. It was `-1.0` once, which is
  positive feedback: autonomous never finished.

## Constants you must not invent

`ShootingConstants.TAG_FIELD_POSES` and `GOALS` are **placeholders that were made
up**. They are not from a game manual. This environment's network policy blocks
`firstinspires.org`, `gm0.org` and `docs.limelightvision.io`, so they cannot be
checked from here.

If you touch them, say plainly that they are unverified. Do not describe an
invented number as sourced, and do not repeat an earlier claim of verification you
cannot support — there is one such claim already in `FieldSymmetry`'s comments, and
it is unsourced.

## The loop budget

Every gain here was tuned around a ~20 ms loop (`LoopTimer.DEFAULT_BUDGET_SECONDS`).
Nothing warns you when that slips, which is why `LoopTimer` runs in every
`CommandOpMode`.

If you add anything to the control loop, measure what it costs and put the number
in the commit message. Concretely:

- No file I/O, formatting, or allocation in the loop. `MatchLog` copies into a
  pre-allocated ring and a background thread writes it — 5.27 µs per row, measured
  over a full simulated match.
- A hardware read is not a field read. `BatteryMonitor` polls on a 250 ms interval
  because reading a `VoltageSensor` on a Control Hub is a bus transaction.
- When something cannot keep up, **drop and count**. Never block the loop, and
  never queue without a bound. A log with holes you know about beats a log that
  cost you the match.

## Things that are deliberately not done

Do not "fix" these without a reason better than symmetry:

- **The shot map is not scaled by voltage.** The flywheel runs closed-loop —
  `setVelocity` plus the controller's PIDF holds rpm — so scaling the setpoint
  compensates twice and makes the commanded rpm wrong. Voltage is *reported*, and
  the shooter says when it is genuinely short of its setpoint.
- **No static singletons.** An FTC app runs many OpModes in one process, so a
  static scheduler or subsystem registry carries stale hardware handles into the
  next run. The scheduler is per-OpMode and reset on exit.
- **No FTCLib / SolversLib dependency.** The command framework here is deliberately
  its own, shaped like WPILib.
- `RobotClock` is the only time source. Do not call `System.nanoTime()` in robot
  code: the offline simulation steps time in realistic slices, and a wall clock
  hands every controller a dt of microseconds, after which the sim is telling you
  about the sim.

## Writing code here

Match the surrounding code: comments explain **why**, and usually name the failure
mode being prevented, rather than restating what the line does. A constant that was
measured says what it was measured with. A guard says what goes wrong without it.

Docs live next to what they describe, and `MANUAL.md` is written for a student who
has not read the code. If a change makes a doc wrong, the change is not finished.

## Commits

Work on the branch you were given; never push elsewhere. Commit messages here are
prose that says what was wrong and how it is known to be fixed, with the measured
numbers in them. Do not put a model name or identifier in a commit message, a PR
body, or a code comment.
