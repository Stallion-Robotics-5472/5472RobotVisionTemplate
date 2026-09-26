# Shoot On The Move

Aiming and shooting while the robot is still driving, on a robot with **no
turret** — so the robot itself has to pivot.

Ported from team 5472's FRC `BOT2-AK` turret solution. See
[`../lib/command/COMMANDS.md`](../lib/command/COMMANDS.md) for the command
framework that drives it.

---

## Why you cannot just aim at the goal

A game piece leaving a moving robot **keeps the robot's velocity**. Shoot
straight at the goal while strafing right and the piece drifts right and misses.

The miss is not subtle. It is almost exactly:

```
    miss ≈ robot speed × time of flight
```

At 60 in/s with a half-second flight, that is **30 inches**. The offline suite
measures 48 inches of miss at the velocities it sweeps. This is the single
biggest reason a robot that shoots beautifully standing still cannot score while
moving.

## The fix: aim at a virtual goal

Aim at a point offset from the real goal by exactly what the robot's motion will
add to the shot:

```
    virtualGoal = goal − shooterVelocity × timeOfFlight
```

```
                         ●  goal
                        ╱│
          aim here ──► ○ │ ← the piece drifts this way
         virtual goal  ╱ │   because the robot is moving
                      ╱  │
                     ╱   │
                    ▭────┘
                  robot, moving right
```

That definition is circular — flight time comes from the shot distance, which
depends on where the virtual goal is, which depends on the flight time — so it is
solved by **iterating to a fixed point**. Three or four passes converge.

> The FRC original ran a fixed three passes. That leaves the reported distance
> and the virtual goal derived from *slightly different* flight times, and the
> inconsistency alone was worth **1.6 inches** of miss in simulation. This
> version iterates until the distance stops moving
> (`AIM_CONVERGENCE_TOLERANCE_IN`).

## What changes without a turret

| | FRC (turret) | Here (no turret) |
|---|---|---|
| Output | turret angle, robot-relative | **field-relative robot heading** |
| Limits | clamped to mechanical sweep | none — the robot spins freely |
| Who drives | chassis independent of aiming | **driver keeps translation, aiming owns heading** |
| Tracking | turret snaps quickly | robot is slow — **feedforward is essential** |

### The angular feedforward

This is the part that makes a turretless SOTM actually work.

While the robot translates past the goal, the direction to the goal **keeps
sweeping**. A heading controller that only sees position error is always
reacting to a target that has already moved, so it permanently trails and the
robot shoots behind. `AimSolution.headingFeedforwardRadPerSec` is the rate the
aim is sweeping at; `DriveSubsystem.driveWithHeadingLock` adds it to the PID
output so the PID only has to clean up what is left.

A turret is fast enough to mostly get away without this. A robot is not.

> The tidy closed-form expression for a static target,
> `(vx·dy − vy·dx) / d²`, is **not** right here: the virtual goal drifts too as
> the shot distance changes, and ignoring that was off by up to 17% in testing.
> `AimLogic` instead re-solves the geometry a millisecond later and differences
> the two aim angles, which captures every term exactly and costs only a few map
> lookups.

### Other things `AimLogic` accounts for

- **Phase delay** — the pose is read, the loop finishes, the command reaches the
  motors, and only then does the robot respond. `PHASE_DELAY_SECONDS` aims at
  where the robot *will be*.
- **An off-centre shooter** — a shooter away from the turn centre both sits
  somewhere else and is *swung sideways* when the robot rotates. That motion goes
  into the piece too (the `ω × r` term). This is a classic cause of a robot that
  shoots well standing still and misses while turning.
- **A shooter that does not fire forwards** — `SHOOTER_YAW_OFFSET_DEG` rotates
  the robot so the *shooter* faces the goal, not the front bumper.

---

## The shot table

`ShooterMap` maps **distance → flywheel rpm, hood angle, flight time**,
interpolating between measured rows and clamping outside them (never
extrapolating — a shooter curve guessed past your data is confident nonsense).

```java
ShooterMap map = ShooterMap.builder()
        //     distance(in)   rpm   hood(deg)  flight(s)
        .add(           18,  1900,     20.0,      0.26)
        .add(           36,  2350,     27.0,      0.40)
        .add(           54,  2800,     32.0,      0.53)
        .build();
```

**Rows are added whole, on purpose.** The FRC version kept three separately-keyed
maps (rpm, hood, flight time) and nothing stopped one of them gaining a data
point the others lacked — the three curves would then interpolate from different
distances and the shot would drift for no visible reason. Here a row is one call,
so they cannot come apart.

**Distances are INCHES.** The FRC original was in metres. Mixing them puts every
shot wildly off.

### Flight time is the column teams skip

It is also the one SOTM depends on. **With flight time left at zero the moving
correction computes a zero offset and does nothing at all** — the robot will
shoot well standing still and miss while moving, with no error message.

You cannot read it off the robot. Time it from a slow-motion phone clip of the
shot. Even a rough value beats zero by a mile.

---

## Setting it up

Everything lives in [`ShootingConstants.java`](ShootingConstants.java), in the
order you should fill it in.

### 1. The goals — derive them from the AprilTags

The tags are already surveyed into the field frame — that is what makes
localization work — so **a tag beside a goal already tells you where that goal
is**. Enter the tag poses once and derive the goals from them:

```java
public static final Map<Integer, Pose2d> TAG_FIELD_POSES = TagGoals.tagTable(
        //        id,            x,     y,  facing (deg)
        11, TagGoals.tagAt(  -36.0,  66.0,  -90.0),
        21, TagGoals.tagAt(   36.0,  66.0,  -90.0));

public static final Goal[] GOALS = TagGoals.from(TAG_FIELD_POSES)
        .goal("hive").fromTag(11).outward(6.0).radius(7.0).worth(5)
        .goal("flower").fromTag(21).outward(6.0).radius(9.0).worth(2)
        .build();
```

Why derive rather than type coordinates twice: a typo in a hand-entered goal
coordinate is **silent**. Aiming is confidently wrong, the pose estimate looks
fine, and nothing flags it. A wrong *tag* pose, by contrast, makes localization
and aiming disagree with each other in a way you can see.

**What deriving does not save you.** It cannot invent the offset from the tag to
the point the piece must pass through, because only the field drawings know that —
a tag on a goal's face is not at the middle of its opening. So you still supply:

| | |
|---|---|
| `outward(n)` | *n* inches in front of the tag's face, along the way it faces (toward the robot) |
| `alongFace(n)` | *n* inches sideways along the face, positive to the tag's left |

That is a ruler measurement off a drawing rather than a coordinate conversion.

`fromTags(2, 3)` averages two tags, for a goal flanked by one either side — the
position lands between them and **both** tags identify it.

Mistakes are loud: an unknown tag ID or a derived goal with no tag throws at
construction rather than aiming somewhere wrong.

A goal with no tag at all can still be declared the long way with
`Goal.named("x", xIn, yIn)` and mixed into the same array.

### Checking the result

Whichever way you enter them, verify with **Shooter Map Tuning**: park somewhere,
compare the `DISTANCE` readout to a tape measure, and adjust until they agree.

### 1b. If you enter coordinates directly instead

```java
Goal.named("primary", 0.0, 60.0)
        .tags(11, 12)
        .radius(6.0)
        .worth(1)
        .build()
```

**The shipped values are placeholders and will aim at the wrong place.** They cannot
be guessed — read them off the season's Competition Manual and field drawings. For
each goal you need:

- **Position**, in this template's frame (origin at field centre, +X right, +Y away
  from the audience, inches). Aim at the point the piece must pass *through* — the
  middle of the opening, not of the structure.
- **Tag IDs** on or beside it, for `TAG_VISIBLE` selection.
- **Its own shot map**, if its height differs from the others.
- **What it scores**, if you want `BEST_VALUE` to use it.

To check a position: run **Shooter Map Tuning**, park somewhere, and compare the
`DISTANCE` readout against a tape measure to that goal. Adjust until they agree.

`radius` is the effective half-width of the opening and sets the heading
tolerance. Use less than the true half-width: the piece has size, the pose has
error, the shot has spread. Two thirds is a fair start.

### 2. Where the shooter is on your robot

`SHOOTER_FORWARD_OFFSET_IN`, `SHOOTER_LEFT_OFFSET_IN`, `SHOOTER_YAW_OFFSET_DEG`.
Measure from the robot's centre of rotation. See the `ω × r` note above for why
this matters more than it looks.

### 3. The shot table

Build it with **Shooter Map Tuning** (below). Five or six rows across your usable
range is plenty.

### 4. Keep the shot range inside the table

`MIN_SHOT_DISTANCE_IN` / `MAX_SHOT_DISTANCE_IN` default to matching the table
exactly. A shot is in range only when **both** the window and the table allow it,
so widening these past the table does not extend the robot's reach — it just
makes the numbers lie. Widen the table first. The offline suite asserts this.

---

## Building the shot table: Shooter Map Tuning

An OpMode (group `Setup`) ported from the FRC `MAP_TUNING` state. The robot
**auto-aims the whole time**, so the distance readout is always honest, but the
shot map is **bypassed** and rpm/hood come from what you dial in.

1. Check `DISTANCE` against a tape measure first. If they disagree, stop —
   the goal's position or the start pose is wrong and nothing else will work.
2. Park at a distance. Hold **LB** to aim; let it settle.
3. `dpad up/down` for rpm, `dpad left/right` for hood. **RT** to fire.
4. When shots go in, press **A** to log the row. It appears on screen as a
   pasteable `.add(...)` line.
5. Move, repeat, then paste the rows into `SHOT_MAP`.
6. Fill in flight time from slow-motion video.

The logged distance is the **standing** distance, not the moving effective
distance — a tuning shot should be taken stationary, and the standing distance is
the honest key.

---

## Using it in TeleOp

`ShootOnTheMoveTeleOp` (group `Drive`) is the reference wiring. The essentials:

```java
aimAndShoot = new AimAndShootCommand(
        drive, shooter,
        () -> -gamepad1.left_stick_y,       // driver keeps translation
        () -> -gamepad1.left_stick_x,
        () -> alliance,                      // for flipping the goal
        () -> gamepad1.right_trigger > 0.5); // fire when ready
whileHeld(() -> gamepad1.left_bumper, aimAndShoot);
```

Hold **LB**: the robot keeps driving wherever the driver puts it while rotating
to lead the goal. Pull **RT**: it feeds as soon as the shot is genuinely ready.
Release LB and heading returns to the driver — the command requires both
subsystems, so both default commands resume automatically.

### When it will not fire

`AimAndShootCommand` gates the feeder on **all** of:

| Condition | Why |
|---|---|
| shot in range and inside the table's data | outside it the setpoint is a guess |
| robot pointed within the tolerance | the tolerance already scales with distance |
| flywheel at speed | feeding during spin-up gives a short shot *and* wrecks the next one |
| **pose trusted** | see below |

`getStatus()` returns one line saying which of these is blocking, so a robot that
will not shoot tells you why instead of sitting there.

### Why pose trust is in that list

Aiming is only as good as the pose, and **a pose seeded 180° out looks perfectly
healthy from the inside** — odometry is self-consistent, the numbers are
plausible, and the robot confidently shoots at empty field.

So the command refuses to fire until MegaTag1 (which is solved without the gyro)
has vouched for the heading. That is the same `isHeadingTrusted()` gate the
localization layer uses; see
[`../LOCALIZATION.md`](../LOCALIZATION.md#catching-a-flipped-start-pose). With
vision disabled the check cannot be made, so it passes and the shot rests on the
seeded pose — as it must, since there is nothing else to go on.

---

## Choosing which goal to shoot at

BIOBUZZ scores on **hives and flowers**, so there is more than one thing to shoot
at. `GoalSelector` picks one each loop.

A `Goal` is more than a coordinate:

```java
Goal.named("high", 0.0, 60.0)
        .tags(21, 22)          // AprilTags that identify it
        .radius(8.0)           // its opening's half-width
        .worth(5)              // points, for the best-value strategy
        .map(HIGH_SHOT_MAP)    // its OWN curve -- a different height needs one
        .build();
```

Two of those deserve a note. **Tags** are how the robot tells which goal it is
looking at. And a goal at a different **height needs its own shot table** — one
flywheel/hood curve cannot serve two heights, so `Goal.map(...)` overrides the
shared default and `AimAndShootCommand` looks the shot up in the selected goal's
map.

### Strategies

| Strategy | Picks |
|---|---|
| `FIXED` | one goal, chosen by the driver or pinned in code |
| `NEAREST` | the closest |
| `LEAST_ROTATION` | whichever needs the least turning from where you point now |
| **`TAG_VISIBLE`** | goals whose AprilTags the camera can currently identify, ties broken by point value then distance |
| `BEST_VALUE` | the highest-value goal actually in range |

`TAG_VISIBLE` is the default. If no goal's tags are in frame it **falls back to
`NEAREST`** rather than refusing to aim — not seeing a tag usually just means the
camera is pointed somewhere else, which is no reason to give up.

### Tag semantics — check the manual

```java
public static final GoalSelector.TagMeaning GOAL_TAG_MEANING =
        GoalSelector.TagMeaning.VISIBLE_MEANS_AVAILABLE;
```

If tags simply sit beside each goal, seeing one identifies it — that is
`VISIBLE_MEANS_AVAILABLE`. If instead a tag gets **covered** as its goal fills up
or is claimed, then the *hidden* tag marks the available goal, and you want
`HIDDEN_MEANS_AVAILABLE`. Get it backwards and the robot prefers exactly the wrong
goals. Read the Competition Manual and set it to match.

### Why switching is deliberately sluggish

This is the part worth understanding. On a turretless robot the selected goal sets
**the whole chassis heading**. AprilTag visibility flickers constantly while
driving — a tag clips the edge of frame, a game piece passes in front of it — and a
selector that switched on a single frame would swing the robot back and forth
between two headings and never settle enough to shoot at either. A turret could
absorb that. A chassis cannot.

So a switch has to be *earned*: a challenger must win `GOAL_SWITCH_FRAMES`
consecutive updates (default 12, about a fifth of a second) before it takes over,
and selection is **frozen outright while a shot is being fed**, so the target
cannot change out from under a shot in progress. The cost is a fraction of a
second of staleness after the picture genuinely changes — far cheaper than
oscillation.

The suite tests exactly this: a single flickering frame must not switch, an
intermittent challenger must *never* take over, and a frozen selector must not
change at all.

### Driver override

The camera does not know everything the driver can see:

| Control (in `ShootOnTheMoveTeleOp`) | Does |
|---|---|
| `A` | cycle the target goal, pinning it |
| left stick button | hand choice back to automatic selection |

Telemetry shows the current goal, why it was chosen, any challenger and its
streak, and which tags are in frame.

### A single-goal game

Leave one entry in `GOALS` and selection is a no-op — every strategy returns it.
No need to change anything else.

---

## Alliances

Goals are written for one alliance and flipped at run time. `GoalSelector` does it
for you:

```java
Translation2d goal = selector.getTargetPosition(alliance);
```

Nothing else changes. The field frame is absolute, the shot table is
alliance-independent, and the aiming maths never knows which alliance it is
playing. The suite checks that a fully mirrored situation (robot pose, velocity
and goal) produces the exactly mirrored aim.

---

## Simulated matches

`./tools/verify/run.sh` plays a **full red match and the mirrored blue match** with
the real subsystems driving a simulated robot: mecanum forward kinematics from the
actual motor powers, accumulating odometry drift, a camera with a real field of
view that only sees tags when a goal is actually in frame, and time stepped in 20 ms
slices so every controller gets the `dt` it will see on the field.

It checks per match that vision actually corrects drift, that the pose and heading
stay accurate, that the robot fires, that it **never** fires outside its own aim
tolerance or outside the shot table's data, and that the target does not thrash.
Then it checks red against blue: the mirror residual, the final headings, shot
counts and goal choice. They currently mirror to **0.000 in**, which is the evidence
that nothing is alliance-dependent that should not be.

Three failure scenarios run too:

| Scenario | What must happen |
|---|---|
| Robot placed backwards (180° seed) | flagged suspect, **never fires while wrong**, recovers on `seedFromVision()` |
| No Limelight at all | still shoots — the pose-trust gate must not deadlock without a camera |
| Camera lost mid-match | target does not thrash, keeps shooting on odometry, recovers when it returns |

### One thing the sim surfaced

If the robot's **start heading does not put a goal tag in the camera's view**, the
start-pose check cannot run at init, so the pose-trust gate keeps the shooter
blocked until the robot first turns toward a goal. It recovers by itself, but you
lose the pre-match warning that would have caught a wrong alliance button. Orient
the robot — or the camera — so a tag is visible while sitting on the field.

## How it is verified

`./tools/verify/run.sh` tests the aiming maths by **simulating the game piece**
rather than asserting on intermediate numbers — the solution is only correct if a
piece launched along it actually lands in the goal. It launches from the shooter
at the speed the table implies, adds the robot's velocity, flies for the table's
flight time, and measures where it lands.

Across 151 swept combinations of position, heading and velocity:

| | worst miss |
|---|---|
| with the moving-shot correction | **0.0018 in** |
| aiming straight at the goal | **47.99 in** |

It also checks the feedforward against a numerical derivative of the aim angle,
that the iteration converged, that a strafing robot leads the correct way, that
the tolerance tightens with distance, that an out-of-range shot is refused, that
a rotating robot with an offset shooter still gets a correction, and that the
alliance flip mirrors cleanly.

None of that is a substitute for shooting real pieces on a real field — it only
proves the maths does what it claims.

---

## Tuning checklist

| Symptom | Look at |
|---|---|
| Good standing, misses while moving | flight time is probably 0 in the table |
| Good standing, misses while turning | `SHOOTER_FORWARD/LEFT_OFFSET_IN` |
| Robot points 180° wrong | `SHOOTER_YAW_OFFSET_DEG` |
| Trails a moving aim, always behind | raise `PHASE_DELAY_SECONDS`; check `TURN_POWER_PER_RAD_PER_SEC` in `DriveSubsystem` |
| Rotation oscillates while locked | `HEADING_kP` / `HEADING_kD` in `PathConstants` |
| Never fires | read `getStatus()` on the telemetry line |
| Fires but shots are short | flywheel not actually at speed — tighten `FLYWHEEL_TOLERANCE_RPM`, tune `FLYWHEEL_F` |
| Distance readout disagrees with tape | the goal's position, or the seeded start pose |
| Robot swings between two goals | `GOAL_SWITCH_FRAMES` too low |
| Always aims at the wrong goal | `GOAL_TAG_MEANING` inverted, or wrong tag IDs |
| Shot is wrong on one goal only | that goal needs its own `.map(...)` |
