# `frc.robot.util` — Utility Reference

This document walks through every class in [`src/main/java/frc/robot/util`](../src/main/java/frc/robot/util). Each entry
gives a plain-language description, a real usage snippet pulled from the subsystems that consume it, and the trade-offs
(upsides / downsides) of the design.

The utilities fall into a few loose buckets:

| Bucket                      | Classes                                                                                                                                                                                                 |
| --------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Telemetry & timing**      | [`LoggedTracer`](#loggedtracer), [`TracerSentinel`](#tracersentinel), [`LoggedCanivore`](#loggedcanivore), [`LoggedSetpointTracker`](#loggedsetpointtracker), [`RollerMechanism2D`](#rollermechanism2d) |
| **Tuning & config**         | [`LoggedTunableNumber`](#loggedtunablenumber)                                                                                                                                                           |
| **Phoenix / hardware glue** | [`PhoenixUtil`](#phoenixutil), [`ComponentStatus`](#componentstatus), [`GatedAlert`](#gatedalert)                                                                                                       |

---

## `LoggedTracer`

**What it is.** A tiny static stopwatch for measuring how long chunks of the main loop take. You call `reset()` once at
the top of the loop, then `record("epochName")` after each phase; each `record` logs the milliseconds elapsed since the
previous `reset`/`record` under `LoggedTracer/<epochName>MS` and rolls the clock forward. Borrowed from FRC 6328
(Mechanical Advantage).

**How it's used.** `Robot.robotPeriodic()` brackets the expensive phases of the loop, and each subsystem closes its own
`periodic()` with a `record` call so its cost shows up as its own epoch:

```java
// Robot.java
LoggedTracer.reset();
PhoenixUtil.refreshAllStatusSignals();
LoggedTracer.record("PhoenixRefresh");
// ... command scheduler runs ...
LoggedTracer.record("Commands");
LoggedCanivore.updateCanivoreStatuses();
LoggedTracer.record("Canivore");

// FlywheelSubsystem.periodic() — last line
LoggedTracer.record("Flywheel");
```

**Upsides.**

- Effectively free to add — one line per measurement point, no objects to thread around.
- Output lands straight in AdvantageScope, so loop-overrun regressions are easy to spot.

**Downsides.**

- A _single_ shared static clock. Interleaving timers (start one before another finishes) is impossible — every `record`
  measures from the last call, full stop.
- Not thread-safe; intended for the main loop only.
- Easy to misread: an epoch's time includes _everything_ since the previous `record`, so a forgotten `record` silently
  folds two phases into one.

---

## `TracerSentinel`

**What it is.** An otherwise-empty `SubsystemBase` whose only job is to call `LoggedTracer.record("SchedulerStart")`
from its `periodic()`. Because the `CommandScheduler` runs subsystem `periodic()` methods in registration order,
constructing this **first** makes it run first — capturing the scheduler's own pre-subsystem overhead as the
`SchedulerStartMS` epoch.

**How it's used.** Declared as the first subsystem field in `RobotContainer` so it registers before anything else:

```java
// RobotContainer.java
private final TracerSentinel tracerSentinel = new TracerSentinel();
```

**Upsides.**

- Turns "scheduler glue cost" into a measurable, named epoch with zero changes to the scheduler.
- Self-documenting: the class exists only to mark a point in time.

**Downsides.**

- Correctness depends entirely on **construction order** — move the field and the measurement silently becomes
  meaningless. The dependency is implicit and easy to break in a refactor.
- Registers a real (if inert) subsystem with the scheduler purely for a side effect.

---

## `LoggedCanivore`

**What it is.** A `CANBus` subclass that polls a CANivore's health (`CANBusStatus`) on a background daemon thread every
500 ms and exposes it for logging. A static registry tracks every instance so one call logs them all. Idea borrowed from
FRC 6328.

**How it's used.** One instance per physical bus is created in `RobotContainer`, passed down into the IO layers that
need the bus, and flushed to the log once per loop from `Robot`:

```java
// RobotContainer.java
private final LoggedCanivore swerveCan = new LoggedCanivore(CANType.SWERVE);
private final LoggedCanivore mechCan   = new LoggedCanivore(CANType.MECH);

// FlywheelIOTalonFX.java — the IO uses the bus name + registers its signals on it
public FlywheelIOTalonFX(LoggedCanivore canivore) { ... }
PhoenixUtil.registerSignals(canivore.getCanType(), allSignals);

// Robot.java — once per loop, log every bus's utilization / error counts
LoggedCanivore.updateCanivoreStatuses();
```

It rejects `CANType.RIO` in the constructor since the native RIO bus isn't a CANivore.

**Upsides.**

- Reading CAN bus status is relatively slow; doing it off-thread keeps it out of the 20 ms loop.
- Logs the metrics that actually predict CAN trouble — `BusUtilization`, `BusOffCount`, `TxFullCount`, `REC`/`TEC`.
- The static registry means callers never have to hold references just to log them.

**Downsides.**

- The polling thread runs `while (true)` with no shutdown path — it only dies because it's a daemon. Constructing
  throwaway instances would leak threads.
- Status is up to 500 ms stale; fine for health trends, not for fast reactions.
- `canivoreStatus` is `volatile` (good), but there's no coordination beyond that — a reader can see a status from one
  poll and the next field from the same poll, which is fine here but worth knowing if the struct grows.
- Static registry = process-global state; awkward for unit tests.

---

## `LoggedSetpointTracker`

**What it is.** A per-mechanism bookkeeper that remembers the last commanded setpoint **per control mode** (velocity,
position, voltage, …) plus which mode is currently active, and logs them with mode-appropriate unit suffixes (`_rps`,
`_rot`, `_v`, `_amps`). Modes are registered up front; `Disabled`/`Follower` are rejected because they have no setpoint.

**How it's used.** A subsystem owns one, updates it whenever it issues a command, and dumps it all in `periodic()`:

```java
// FlywheelSubsystem.java
private final LoggedSetpointTracker setpointTracker = new LoggedSetpointTracker(
    "Flywheel/Setpoints", MotorControlMode.DutyCycle, MotorControlMode.Voltage, MotorControlMode.Velocity);

setpointTracker.updateSetpoint(commandedVelocitySetpoint, MotorControlMode.Velocity); // also sets active mode
setpointTracker.setControlMode(MotorControlMode.Disabled);                            // on stop
setpointTracker.logAll();                                                             // in periodic()
```

When logging, only the _active_ mode's setpoint is emitted as its real value; inactive modes log `0.0`, so a graph never
shows a stale velocity target while the mechanism is actually holding position.

**Upsides.**

- Centralizes "what did we last ask for, and in what mode" — no scattering of ad-hoc logging strings across the
  subsystem.
- Unit suffixes make the AdvantageScope keys self-describing.
- Zeroing inactive modes prevents misleading stale-setpoint traces.

**Downsides.**

- Setpoints are all `double` — no compile-time unit safety, so it's on the caller to pass the right units for the mode.
- `requireRegistered` throws at runtime if you log/update an unregistered mode; mistakes surface as crashes rather than
  compile errors.
- One tracker only models one active mode at a time; a mechanism blending modes wouldn't fit.

---

## `RollerMechanism2D`

**What it is.** Builds a `LoggedMechanism2d` that draws a spinning roller — a radial "spike" ligament with a polygon of
edge ligaments forming the rim — so a rotating wheel's position is visible in AdvantageScope/Glass.
`setPosition(rotations)` spins the spike.

**How it's used.** A subsystem with a roller-like mechanism owns one and feeds it the measured position each loop:

```java
// FlywheelSubsystem.java
private final RollerMechanism2D mechanism = new RollerMechanism2D(0.4);
...
mechanism.setPosition(inputs.positionRot); // in periodic()
```

(Also used by `RollerSubsystem` and `TowerSubsystem`.) The polygon is intentionally coarse on the real robot
(`ROLLER_SIDES == 3` when `CURRENT_MODE == REAL`, 20 in sim) to keep AdvantageKit serialization cheap on the RIO.

**Upsides.**

- Gives otherwise-invisible spinning mechanisms a real visual on the dashboard.
- The REAL-vs-sim side count is a thoughtful perf trade — detail where you can afford it.
- Several constructors (colors, radius, defaults) make it a one-liner at call sites.

**Downsides.**

- Purely cosmetic telemetry; it serializes a small mechanism every loop for visualization only.
- The geometry math (edge length, turn angle, first-edge offset) is fiddly and bespoke — not obvious to modify without
  re-deriving it.
- `setPosition` takes rotations and converts to degrees internally; passing degrees by mistake is an easy, silent bug.

---

## `LoggedTunableNumber`

**What it is.** A `double` value that comes from the dashboard (NetworkTables `/Tuning/...`) when
`Constants.TUNING_MODE` is on, and falls back to a hard-coded default otherwise. Implements `DoubleSupplier`. A nested
`Watcher` lets you run code only when one of a group of tunables changes — the backbone of live PID tuning. Adapted from
FRC 6328.

**How it's used.** Subsystems expose their gains as tunables, group them in a `Watcher`, and re-push to hardware only on
change:

```java
// FlywheelSubsystem.java
private final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", pid.kP());
// ... kI, kD, kS, kV, kA ...
private final LoggedTunableNumber.Watcher pidWatcher = LoggedTunableNumber.watch(kP, kI, kD, kS, kV, kA);

private final LoggedTunableNumber.Watcher motionMagicWatcher =
    LoggedTunableNumber.watch(motionMagicAccel, motionMagicVelo, motionMagicJerk);

// periodic():
pidWatcher.ifChanged(values -> io.setPID(...));        // values[] = each tunable, in order
motionMagicWatcher.ifChanged(() -> io.setMotionMagic(...));
```

`ifChanged` is a no-op outside `TUNING_MODE`, so production builds never pay for the comparison or the reconfigure.

**Upsides.**

- Single source of truth for a constant: live-tunable in the pit, baked-in default in a match.
- The `Watcher` makes "reconfigure hardware only when a gain actually moved" trivial and cheap — important because
  applying configs to a TalonFX is expensive.
- `ifChanged(Consumer<double[]>)` hands you all the current values in registration order, so the callback doesn't have
  to re-read each tunable.

**Downsides.**

- A tunable with no default returns `0.0` from `get()` — forget `initDefault` and you silently get zero gains.
- Change detection is exact `!=` on doubles; fine for dashboard-entered values, but not something to rely on for
  computed inputs.
- `Watcher.last` is seeded at construction. If the dashboard value differs from the default at startup, the first
  `ifChanged` may or may not fire depending on timing — order your construction vs. first poll carefully.
- The order of arguments to `watch(...)` must match how the callback indexes `values[]`; a reordering bug is silent.

---

## `PhoenixUtil`

**What it is.** A grab-bag of static helpers for CTRE Phoenix 6, from FRC 6328:

- `tryUntilOk(maxAttempts, command [, alert])` — retry a config call until it returns `OK`, optionally raising an
  `Alert` on final failure.
- `registerSignals(canType, ...)` / `refreshAllStatusSignals()` — collect every status signal, keyed by bus, and refresh
  them all in one synchronized batch per loop.
- `toMotorControlMode(...)` / `toEncoderHealth(...)` — map Phoenix's hardware-specific enums onto the hardware-agnostic
  [`ComponentStatus`](#componentstatus) enums.

**How it's used.** The IO layer leans on it heavily — retried config, batched refresh, enum mapping:

```java
import static frc.robot.util.PhoenixUtil.tryUntilOk;

tryUntilOk(5, () -> leader.getConfigurator().apply(config), failedToConfigureLeaderAlert);
tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(100.0, allSignals), failedToSetFrequencyAlert);
PhoenixUtil.registerSignals(canivore.getCanType(), allSignals);
inputs.controlMode = PhoenixUtil.toMotorControlMode(controlMode.getValue());

// Robot.java — one batched refresh of every registered signal, every loop
PhoenixUtil.refreshAllStatusSignals();
```

**Upsides.**

- `tryUntilOk` turns flaky CAN config (common at boot) into a retry-with-alert one-liner — and it composes directly with
  [`GatedAlert`](#gatedalert).
- Batching all signals into one `refreshAll` per bus is the right call for loop timing: one synchronized CAN round-trip
  instead of N.
- The enum mappers decouple subsystem logic from Phoenix's sprawling `ControlModeValue` / `MagnetHealthValue` enums, so
  the rest of the code speaks a small, stable vocabulary.

**Downsides.**

- `registerSignals` rebuilds the whole array via `System.arraycopy` on every call — fine at init time (where it's only
  called during construction), but O(n²) if anyone called it in a loop.
- The signal registry is static/global, so signals registered in tests or across robot reconstructions accumulate unless
  the process restarts.
- `toMotorControlMode`'s `default` falls through to `Disabled`; a brand-new Phoenix control mode would be silently
  misreported rather than flagged.
- `tryUntilOk` only retries on a non-OK `StatusCode` — it won't catch a thrown exception.

---

## `ComponentStatus`

**What it is.** A namespace holding two hardware-agnostic enums: `MotorControlMode` (`Disabled`, `Follower`,
`DutyCycle`, `Voltage`, `TorqueCurrent`, `Position`, `Velocity`) and `EncoderHealth` (`Good`, `Marginal`, `Bad`,
`Unknown`). These are the vocabulary the rest of the robot uses instead of vendor enums.

**How it's used.** It's the common currency between [`PhoenixUtil`](#phoenixutil) (which produces these from Phoenix
enums), [`LoggedSetpointTracker`](#loggedsetpointtracker) (which keys setpoints by `MotorControlMode`), and the
IO/subsystem layers:

```java
// in an IO layer
inputs.controlMode = PhoenixUtil.toMotorControlMode(controlMode.getValue());
inputs.encoderHealth = PhoenixUtil.toEncoderHealth(magnetHealth.getValue());

// in a subsystem
setpointTracker.updateSetpoint(speed, MotorControlMode.DutyCycle);
```

**Upsides.**

- One vendor-neutral enum set means swapping motor controllers (or running sim) doesn't ripple through every subsystem —
  only the mapping in `PhoenixUtil` changes.
- Small and stable: easy to log, switch on, and reason about.

**Downsides.**

- It's a deliberately _lossy_ abstraction — e.g. all the Motion Magic position variants collapse to `Position`. If you
  ever need to distinguish them downstream, the information is already gone.
- A plain container class with two enums; the `Follower`/`Disabled` special-casing lives in the consumers
  ([`LoggedSetpointTracker`](#loggedsetpointtracker)), not here, so the rules are spread out.

---

## `GatedAlert`

**What it is.** A drop-in `Alert` subclass that only actually raises while a `BooleanSupplier` "gate" is open. Use it to
suppress cascading noise — e.g. silence a motor's config-failure alerts while the motor itself is disconnected (no point
yelling about config when it's just unplugged). The underlying raised state is preserved, so reopening the gate restores
any still-valid alert. Every instance self-registers, and the static `pushAll()` re-evaluates every gate; `Robot` calls
it once per loop so gates are polled automatically.

**How it's used.** The IO layer wires every config/setup alert to a connection flag and just keeps that flag current —
no manual re-pushing, because `Robot.robotPeriodic` re-evaluates all gates each loop:

```java
// FlywheelIOTalonFX.java
private boolean leaderConnected = false;
private final GatedAlert failedToConfigureLeaderAlert =
    new GatedAlert(LEADER_ALERT_PREFIX + "Failed to configure motor", AlertType.kError, () -> leaderConnected);

// keeping the gate field current is enough; pushAll() picks it up next loop:
private void refreshLeaderAlerts(boolean connected) {
    leaderConnected = connected;
    leaderDisconnectedAlert.set(!connected);
}

// composes with tryUntilOk, which calls alert.set(...) on failure:
tryUntilOk(5, () -> leader.getConfigurator().apply(config), failedToConfigureLeaderAlert);

// Robot.robotPeriodic() — re-evaluates every gate once per loop:
GatedAlert.pushAll();
```

**Upsides.**

- Kills alert spam: a disconnected motor produces _one_ "disconnected" alert instead of a dozen downstream "couldn't
  configure / couldn't set follower" alerts.
- Truly drop-in — it `extends Alert`, so it works anywhere an `Alert` is expected, including
  `PhoenixUtil.tryUntilOk(..., alert)`.
- Preserving the underlying state means a transient gate close doesn't lose a real alert.
- Gates are **polled automatically** via `pushAll()` once per loop, so a flip takes effect within one loop and a missed
  flip can't get stuck — callers only have to keep the gate field current.

**Downsides.**

- The per-loop `pushAll()` re-evaluates every registered gate. Each eval is cheap (WPILib's `Alert.set` short-circuits
  when the value is unchanged), but the registry is process-global static state — the same testing-awkwardness noted for
  [`LoggedCanivore`](#loggedcanivore).
- A gate flip can lag by up to one loop. Fine for connection-health alerts; a caller needing a same-loop update can
  still call `push()` explicitly after flipping.
- `set()` and `push()` both funnel through `super.set(actualValue && gate)`; reasoning about the effective state still
  requires holding both the raised state and the gate in your head.

---
