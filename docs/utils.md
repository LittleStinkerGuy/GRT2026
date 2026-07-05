# Utility Reference

This document walks through every util class I developed while rewriting the 2026 GRT robot code. Each doc has a
description, usage snippets pulled from the subsystems that use it, the reasons I created it and I had claude generate
potential downsides.

The utilities fall into a few loose buckets:

| Bucket                      | Classes                                                                                                                                                                                                 |
| --------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Telemetry & timing**      | [`LoggedTracer`](#loggedtracer), [`TracerSentinel`](#tracersentinel), [`LoggedCanivore`](#loggedcanivore), [`LoggedSetpointTracker`](#loggedsetpointtracker), [`RollerMechanism2D`](#rollermechanism2d) |
| **Tuning & config**         | [`LoggedTunableNumber`](#loggedtunablenumber)                                                                                                                                                           |
| **Phoenix / hardware glue** | [`PhoenixUtil`](#phoenixutil), [`ComponentStatus`](#componentstatus), [`GatedAlert`](#gatedalert)                                                                                                       |

---

## `LoggedTracer`

**What it is.** A tool for measuring how long chunks of the main loop take. You call `reset()` once at the top of the
robot periodic, then `record("epochName")` after each phase; each `record` logs the milliseconds since the previous
`reset`/`record`. Borrowed from Mechanical Advantage (Team 6328).

**How it's used.** After each function ran in the robot periodic and at the end of each subsystem periodic, you attached
a `LoggedTracer.record("epochName")` call which tracks the time between the last record call and when this was called.
By adding a tracer call after running the command scheduler, you can find the time it takes to run commands. This is
because the command scheduler runs subsystem periodics first, which have their own tracer calls, and then the commands.

```java
// Robot.java
@Override
public void robotPeriodic() {
    LoggedTracer.reset();

    PhoenixUtil.refreshAllStatusSignals();
    LoggedTracer.record("PhoenixRefresh");

    CommandScheduler.getInstance().run();
    LoggedTracer.record("Commands");
}

// FlywheelSubsystem.java
@Override
public void periodic() {
    // other periodic content
    LoggedTracer.record("Flywheel");
}
```

**Upsides.**

- Effectively free to add - one line per measurement point and no objects created.
- Easy to track sources of loop overruns and compute.

**Downsides.**

- A _single_ shared static clock. Interleaving timers (start one before another finishes) is impossible — every `record`
  measures from the last call, full stop.
- Not thread-safe; intended for the main loop only.
- Easy to misread: an epoch's time includes _everything_ since the previous `record`, so a forgotten `record` silently
  folds two phases into one.

---

## `TracerSentinel`

**What it is.** A `SubsystemBase` whose only job is to call `LoggedTracer.record("SchedulerStart")` from its
`periodic()`. Constructing this first makes it run first, capturing the scheduler's pre-subsystem overhead as the
`SchedulerStartMS` epoch.

**How it's used.** Declared as the first subsystem field in `RobotContainer` so it registers before anything else:

```java
// RobotContainer.java
private final TracerSentinel tracerSentinel = new TracerSentinel();
```

**Upsides.**

- Allows us to measure scheduler overhead and not factor that into an unrelated subsystem.
- One line in code

**Downsides.**

- Correctness depends entirely on **construction order** — move the field and the measurement silently becomes
  meaningless. The dependency is implicit and easy to break in a refactor.
- Registers a real (if inert) subsystem with the scheduler purely for a side effect.

---

## `LoggedCanivore`

**What it is.** A `CANBus` subclass that polls a CANivore's health (`CANBusStatus`) on a background daemon thread every
500 ms and exposes it for logging. A static registry tracks every instance so one call logs them all. Idea borrowed and
modified from Mechanical Advantage (Team 6328).

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
@Override
public void robotPeriodic() {
    LoggedCanivore.updateCanivoreStatuses();
}
```

It rejects `CANType.RIO` in the constructor since the native RIO bus isn't a CANivore.

**Upsides.**

- Reading CAN bus status is relatively slow so doing it off-thread is good
- Logs the metrics that will allow us to debug CAN
- The static registry means callers never have to hold references just to log them.
- Provides the `CANType` that `PhoenixUtil.registerSignals()` groups status signals by

**Downsides.**

- The polling thread runs `while (true)` with no shutdown path — it only dies because it's a daemon. Constructing
  throwaway instances would leak threads.
- Status is up to 500 ms stale; fine for health trends, not for fast reactions.
- `canivoreStatus` is `volatile` (good), but there's no coordination beyond that — a reader can see a status from one
  poll and the next field from the same poll, which is fine here but worth knowing if the struct grows.
- Static registry = process-global state; awkward for unit tests.

---

## `LoggedSetpointTracker`

**What it is.** A per-subsystem helper that remembers the last commanded setpoint per control mode (velocity, position,
voltage, e.g.) plus which mode is currently active, and logs them. When logging, only the active mode's setpoint is
logged as its real value, all other modes log `0.0`. It also exposes an `atSetpoint()` utility to check if subsystem is
at its closed loop setpoint.

**How it's used.** A subsystem initiates one, updates it whenever it issues a command, and logs it in `periodic()`:

```java
// FlywheelSubsystem.java
private final LoggedSetpointTracker setpointTracker = new LoggedSetpointTracker(
    "Flywheel/Setpoints", MotorControlMode.DutyCycle, MotorControlMode.Voltage, MotorControlMode.Velocity);

public void setVelocity(double velocityRPS) {
    setpointTracker.updateSetpoint(commandedVelocitySetpoint, MotorControlMode.Velocity);
}

public void stop() {
    setpointTracker.setControlMode(MotorControlMode.Disabled);
}

public boolean atSetpoint() {
    return setpointTracker.atSetpoint(
        MotorControlMode.Velocity, inputs.velocityRPS, ShooterConstants.Flywheel.VELOCITY_TOLERANCE_RPS);
}

@Override
public void periodic() {
    setpointTracker.logAll();
}
```

**Upsides.**

- Centralizes logging and tracking setpoints
- Unit suffixes make the AdvantageScope keys self-describing.
- Zeroing inactive modes is done by default and prevents misleading stale-setpoint traces.

**Downsides.**

- Setpoints are all `double` — no compile-time unit safety, so it's on the caller to pass the right units for the mode.
- `requireRegistered` throws at runtime if you log/update an unregistered mode; mistakes surface as crashes rather than
  compile errors. (throwing an error _should_ be okay during robot runtime but needs testing)

---

## `RollerMechanism2D`

**What it is.** Builds a `LoggedMechanism2d` that draws a spinning roller so a rotating wheel's position is visible in
AdvantageScope/Glass.

**How it's used.** A subsystem will construct one then call `setPosition(rotations)` to spin the wheel in periodic. In
real robot code, the wheel is rendered as a triangle to save network table, RIO bandwidth, and serialization, but in sim
it is a 20 sided polygon.

```java
// FlywheelSubsystem.java
private final RollerMechanism2D mechanism = new RollerMechanism2D(0.4); // radius = 0.4

@Override
public void periodic() {
    mechanism.setPosition(inputs.positionRot);
}
```

**Upsides.**

- Gives otherwise-invisible spinning mechanisms a real visual on the dashboard.
- Several constructors (colors, radius, defaults) make it a one-liner at call sites.

**Downsides.**

- Purely cosmetic telemetry; it serializes a small mechanism every loop for visualization only.
- The geometry math (edge length, turn angle, first-edge offset) is fiddly and bespoke — not obvious to modify without
  re-deriving it.
- `setPosition` takes rotations and converts to degrees internally; passing degrees by mistake is an easy, silent bug.

---

## `LoggedTunableNumber`

**What it is.** Refined tune this yo. It takes a `double` value that comes from the dashboard (NetworkTables
`/Tuning/...`) when `Constants.TUNING_MODE` is on, and uses a hard-coded constant in matches. The `Watcher` class lets
you run code only when one of a group of tunables changes useful for live PID tuning. Adapted from FRC 6328.

**How it's used.** Subsystems expose their gains as tunables, group them in a `Watcher`, and re-push to hardware only on
change. Individual values can also be fetched with `.get()`

```java
// FlywheelSubsystem.java
private final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", pid.kP());
// ... kI, kD, kS, kV, kA ...
private final LoggedTunableNumber.Watcher pidWatcher = LoggedTunableNumber.watch(kP, kI, kD, kS, kV, kA);

@Override
public void periodic() {
    pidWatcher.ifChanged(
        () -> io.updatePID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get()));
}
```

**Upsides.**

- Single source of truth for a constant which is live-tunable in the shop and fixed in a match.
- The `Watcher` makes retuning configurations trivial.

**Downsides.**

- A tunable with no default returns `0.0` from `get()` — don't put it in the constructor and forget `initDefault` and
  you silently get zero gains.
- Change detection is exact `!=` on doubles; fine for dashboard-entered values, but not something to rely on for
  computed inputs.
- `Watcher.last` is seeded at construction. If the dashboard value differs from the default at startup, the first
  `ifChanged` may or may not fire depending on timing — order your construction vs. first poll carefully.
- The order of arguments to `watch(...)` must match how the callback indexes `values[]`; a reordering bug is silent.

---

## `PhoenixUtil`

**What it is.** Helpers for CTRE Phoenix 6;

- `tryUntilOk()` — retry a config call until it returns `OK`, optionally raising an `Alert` on final failure.
- `registerSignals()` / `refreshAllStatusSignals()` — collect every status signal on a canivore and refresh them all in
  one batch per loop.
- `toMotorControlMode()` / `toEncoderHealth()` — map Phoenix's hardware-specific enums onto the hardware-agnostic
  [`ComponentStatus`](#componentstatus) enums.

**How it's used.** The IO layer leans on it heavily — retried config, batched refresh, enum mapping:

```java
// FlywheelIOTalonFX.java
public FlywheelIOTalonFX(LoggedCanivore canivore) {
    tryUntilOk(5, () -> motor.getConfigurator().apply(config), failedToConfigureMotorAlert);
    PhoenixUtil.registerSignals(canivore.getCanType(), allSignals);
}
@Override
public void updateInputs(FlywheelIOInputs inputs) {
    inputs.controlMode = PhoenixUtil.toMotorControlMode(controlMode.getValue());
}

// Robot.java
@Override
public void robotPeriodic() {
    PhoenixUtil.refreshAllStatusSignals();
}
```

**Upsides.**

- `tryUntilOk` replaces logic that we currently use with a one liner and works with [`GatedAlert`](#gatedalert)
- Batching all signals into one `refreshAll` per bus decreases code execution by a lot of time
- The enum mappers simplify Phoenix's huge `ControlModeValue` / `MagnetHealthValue` enums, so the rest of the code can
  work with simple, hardware agnostic enums.

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
`Unknown`). These are used by the rest of the robot instead of vendor specific enums.

**How it's used.** Used to log hardware status and implement logic.

```java
// FlywheelIOTalonFX.java
@Override
public void updateInputs(FlywheelIOInputs inputs) {
    inputs.controlMode = PhoenixUtil.toMotorControlMode(controlMode.getValue());
    inputs.encoderHealth = PhoenixUtil.toEncoderHealth(magnetHealth.getValue());
}

// FlywheelSubsystem.java
public void setVelocity(double velocityRPS) {
    setpointTracker.updateSetpoint(commandedVelocitySetpoint, MotorControlMode.Velocity);
}
```

**Upsides.**

- One vendor-neutral enum set means swapping motor controllers or running sim doesn't affect every subsystem only the
  mapping in `PhoenixUtil` changes.
- Less terms than phoenix specific enums so it is easy to log and use a switch on.

**Downsides.**

- It's a deliberately _lossy_ abstraction — e.g. all the Motion Magic position variants collapse to `Position`. If you
  ever need to distinguish them downstream, the information is already gone.
- A plain container class with two enums; the `Follower`/`Disabled` special-casing lives in the consumers
  ([`LoggedSetpointTracker`](#loggedsetpointtracker)), not here, so the rules are spread out.

---

## `GatedAlert`

**What it is.** A drop-in `Alert` subclass that only actually raises while a `BooleanSupplier` "gate" is open. Used to
suppress cascading alerts (e.g. silence a motor's config-failure alerts while the motor itself is disconnected). The
underlying raised state is preserved, so reopening the gate restores any still-valid alert.

**How it's used.** The IO layer wires every config/setup alert to a connection flag. Every instance self-registers in a
static registry, and the `pushAll()` method re-evaluates every gate in `robotPeriodic()` automatically.

```java
// FlywheelIOTalonFX.java
private boolean leaderConnected = false;
private final GatedAlert failedToConfigureLeaderAlert =
    new GatedAlert(LEADER_ALERT_PREFIX + "Failed to configure motor", AlertType.kError, () -> leaderConnected);

public FlywheelIOTalonFX(LoggedCanivore canivore) {
  tryUntilOk(5, () -> leader.getConfigurator().apply(config), failedToConfigureLeaderAlert);
}

// Robot.java
@Override
public void robotPeriodic() {
    GatedAlert.pushAll();
}
```

**Upsides.**

- A disconnected motor produces one "disconnected" alert instead of many downstream "couldn't configure / couldn't set
  follower" alerts.
- It's drop-in because it `extends Alert`, so it works anywhere an `Alert` is expected, including
  `PhoenixUtil.tryUntilOk(..., alert)`.
- Preserving the underlying state means the gate closing won't lose the alert status.
- Gates are polled automatically via `pushAll()` once per loop, so a flip takes effect within one loop and a missed flip
  can't get stuck and callers only have to keep the gate field current.

**Downsides.**

- The per-loop `pushAll()` re-evaluates every registered gate. Each eval is cheap (WPILib's `Alert.set` short-circuits
  when the value is unchanged), but the registry is process-global static state — the same testing-awkwardness noted for
  [`LoggedCanivore`](#loggedcanivore).
- A gate flip can lag by up to one loop. Fine for connection-health alerts; a caller needing a same-loop update can
  still call `push()` explicitly after flipping.
- `set()` and `push()` both funnel through `super.set(actualValue && gate)`; reasoning about the effective state still
  requires holding both the raised state and the gate in your head.

---
