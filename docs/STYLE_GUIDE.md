# Style Guide

---

## Code Style

### Naming

- Method and variable naming
- Adding unit suffixes to variable names

### Formatting & Layout

- Indentation, braces, and line length (defer to the auto-formatter)
- Ordering of members within a type
- Blank-line and grouping conventions

### Imports

- Wildcard import policy
- Static import policy
- Import ordering

### Comments & Documentation

- When documentation comments are required
- Doc-comment style
- Handling of TODOs and dead/commented-out code

### Numeric & Unit Conventions

- Representation of physical quantities (primitive types vs. a units library)
- Where and how conversions happen

## Code Structure

### Project & Package Layout

- Top-level entry points
- One package per subsystem/feature
- Where shared utilities and helpers live

### Subsystems & Hardware Abstraction

- Separating control logic from hardware behind an IO interface
- Real-hardware and simulation implementations behind the same interface
- Confining vendor/hardware types to the hardware layer

### Motor & Servo Mechanisms

- Shared, reusable motor/servo control classes with built-in closed-loop (PID / profiled) control
- Composing mechanisms from these rather than re-implementing control per subsystem
- Configuration and tuning entry points

### Constants & Configuration

- Scoped/split constants grouped per subsystem
- No logic in constants
- Tunable vs. fixed values, and their defaults

### Commands

- Command framework conventions
- Where commands are defined vs. where they are constructed/bound
- Composition, requirements, and keeping commands thin over the subsystem API

### Simulation

- A simulation implementation for every mechanism
- Run-mode selection (real / sim / replay)
- Keeping simulation deterministic and hardware-free

## Shared Utilities & Required Patterns

### Loop-Timing Instrumentation

- A per-subsystem timing marker recorded each loop
- Construction/registration order where it affects what a marker measures

### CAN Bus Health

- A monitored bus representation required for each physical bus
- Passing the bus handle down into the hardware layer that uses it

### Device Configuration

- Bounded retry when applying device configuration (retry until success / fixed attempts)
- Raising an alert on final failure

### Status Signal Management

- Central registration of all device status signals
- A single batched, synchronized refresh per loop (and per bus)

### Vendor-Agnostic Status Mapping

- Mapping vendor-specific states/enums to a shared internal vocabulary

### Tunable Values

- Dashboard-backed tunable values with in-code defaults
- Re-applying to hardware only when a value actually changes

### Setpoint & State Tracking

- Tracking the last commanded setpoint per control mode
- Consistent logging of active vs. stale setpoints

### Mechanism Visualization

- Dashboard visualization of mechanism state
- Detail vs. performance trade-offs across run modes

## Telemetry & Diagnostics

### Logging & Replay

- The logging/replay framework and why it is mandatory
- Logged inputs vs. logged outputs
- Annotation-based auto-logging vs. manual logging calls

### Log Conventions

- Key naming and grouping
- Unit suffixes on log keys
- What should and should not be logged

### Alerts & Fault Handling

- Surfacing faults to the dashboard
- A standard alert message format
- Gating/suppressing redundant or cascading alerts
- Severity levels and when to use each

## Build & Development Environment

### Code Formatting

- Auto-formatter wired into the build (format-on-build)
- The canonical formatter config as the single source of truth
- Don't hand-format against the tool

### Static Analysis

- Linter / style checker as a build gate
- Zero-warning policy
- Documented exclusions
