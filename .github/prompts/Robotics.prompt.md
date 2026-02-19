---
mode: agent
---

# Robotics Subsystem Development Guide

When creating, combining, or refactoring subsystems in this FRC robot project, follow these patterns exactly.

## AdvantageKit IO Layer Pattern

Every hardware mechanism has three layers:

1. **IO interface** (`*IO.java`) — defines `@AutoLog` inputs class and an outputs class, plus default no-op methods
2. **IO implementations** (`*IOTalonFX.java`, `*IOSim.java`) — hardware-specific code
3. **Subsystem class** — mode-agnostic logic consuming the IO interface

### IO Interface Template
```java
// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.<subsystem>.<mechanism>;

import org.littletonrobotics.junction.AutoLog;

public interface <Mechanism>IO {
  @AutoLog
  class <Mechanism>IOInputs {
    public boolean motorConnected = false;
    // sensor readings...
  }

  // Optional: output mode enum if mechanism has multiple control modes
  enum <Mechanism>IOOutputMode {
    CLOSED_LOOP,
    OPEN_LOOP
  }

  class <Mechanism>IOOutputs {
    // setpoints, gains, control mode...
  }

  default void updateInputs(<Mechanism>IOInputs inputs) {}
  default void applyOutputs(<Mechanism>IOOutputs outputs) {}
  default void setBrakeMode(boolean enableBrake) {}
}
```

**Key rules:**
- `@AutoLog` on the inputs class generates `*IOInputsAutoLogged` at build time (not resolvable in IDE until `./gradlew build`)
- Outputs are a plain class (not `@AutoLog`), updated by the subsystem and passed to `applyOutputs()`
- All methods have `default` empty implementations so `new IO() {}` works as a no-op

### IO Implementation (TalonFX) Conventions
- Use `PhoenixUtil.tryUntilOk(5, () -> ...)` for all Phoenix 6 configuration calls
- Cache `StatusSignal` objects as fields; refresh them in `updateInputs()`
- Use `BaseStatusSignal.setUpdateFrequencyForAll()` and `talon.optimizeBusUtilization()`
- Track last-applied PID gains to avoid redundant configurator calls in `applyOutputs()`
- CAN IDs are `private static final int` at the top of the class (with a TODO to move to constants)

### IO Implementation (Sim) Conventions
- Use WPILib simulation classes (`SingleJointedArmSim`, `DCMotorSim`, etc.)
- Use `Constants.loopPeriodSecs` for simulation timestep
- Clamp voltages to `[-12.0, 12.0]` with `MathUtil.clamp()`
- Set `inputs.motorConnected = true` (or equivalent) since sim is always "connected"

## Subsystem Class Pattern

All mechanism subsystems extend `FullSubsystem`, which provides:
- `periodic()` — called by command scheduler (read inputs, process alerts, update gains)
- `periodicAfterScheduler()` — called after scheduler (apply outputs to hardware)

### Single-Mechanism Subsystem Structure
```java
public class <Subsystem> extends FullSubsystem {
  private final <Mechanism>IO io;
  private final <Mechanism>IOInputsAutoLogged inputs = new <Mechanism>IOInputsAutoLogged();
  private final <Mechanism>IOOutputs outputs = new <Mechanism>IOOutputs();

  // Disconnect detection
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert = new Alert("... disconnected!", Alert.AlertType.kWarning);

  // Brake/coast tracking
  @Setter private BooleanSupplier coastOverride = () -> false;
  private Boolean lastBrakeMode = null;
```

### Multi-Mechanism Subsystem Pattern (e.g., Intake = Pivot + Rollers)

When a subsystem has multiple mechanisms that are always coordinated together:

**Directory structure:**
```
subsystems/<subsystem>/
  <Subsystem>.java                  ← single FullSubsystem class
  <mechanism1>/
    <Mechanism1>IO.java
    <Mechanism1>IOTalonFX.java
    <Mechanism1>IOSim.java
  <mechanism2>/
    <Mechanism2>IO.java
    <Mechanism2>IOTalonFX.java
    <Mechanism2>IOSim.java
```

**Subsystem class takes all IO interfaces in constructor:**
```java
public class Intake extends FullSubsystem {
  private final IntakePivotIO pivotIO;
  private final IntakePivotIOInputsAutoLogged pivotInputs = new IntakePivotIOInputsAutoLogged();
  private final IntakePivotIOOutputs pivotOutputs = new IntakePivotIOOutputs();

  private final IntakeRollersIO rollersIO;
  private final IntakeRollersIOInputsAutoLogged rollersInputs = new IntakeRollersIOInputsAutoLogged();
  private final IntakeRollersIOOutputs rollersOutputs = new IntakeRollersIOOutputs();

  public Intake(IntakePivotIO pivotIO, IntakeRollersIO rollersIO) {
    this.pivotIO = pivotIO;
    this.rollersIO = rollersIO;
  }
```

**Key rules for multi-mechanism subsystems:**
- Each mechanism gets its own disconnect `Debouncer` + `Alert`
- Each mechanism gets its own brake mode tracking (`lastPivotBrakeMode`, `lastRollersBrakeMode`)
- `periodic()` updates ALL mechanism inputs and alerts in sequence
- `periodicAfterScheduler()` applies ALL mechanism outputs in sequence
- Log paths use `"Subsystem/Mechanism/..."` hierarchy (e.g., `"Intake/Pivot"`, `"Intake/Rollers"`)
- Provide combined convenience commands (e.g., `deployCommand()` moves pivot AND starts rollers)
- Also provide per-mechanism methods for fine-grained control

## periodic() Checklist
Every `periodic()` must, in this order:
1. `io.updateInputs(inputs)` and `Logger.processInputs("Name", inputs)`
2. Set disconnect alert via `Debouncer` + `Robot.showHardwareAlerts()`
3. Handle brake/coast mode toggling (compare with `lastBrakeMode`, call `io.setBrakeMode()`)
4. Update tunable gains on outputs (`outputs.kP = kP.get()`, etc.)
5. For rollers/simple mechanisms: handle enabled/disabled voltage logic

## periodicAfterScheduler() Checklist
1. Determine control mode based on `DriverStation.isEnabled()`
2. Clamp setpoints (`MathUtil.clamp(goalAngleDeg, min, max)`)
3. Set output mode, position/voltage, and log goals
4. Call `io.applyOutputs(outputs)` for each mechanism

## LoggedTunableNumber Pattern
```java
private static final LoggedTunableNumber kP = new LoggedTunableNumber("Subsystem/kP");

static {
  switch (Constants.getMode()) {
    case REAL, REPLAY -> { kP.initDefault(100); }
    case SIM -> { kP.initDefault(2.0); }
  }
}
```
- Always use `static final` fields
- Use `initDefault()` inside a `static {}` block when values differ per mode
- Pass a default directly in the constructor when the value is the same across all modes:
  `new LoggedTunableNumber("Intake/Rollers/RunVolts", 10.0)`

## RobotContainer Instantiation Pattern

```java
// Field declaration
private Intake intake;

// Inside constructor switch
case COMPBOT -> {
  intake = new Intake(new IntakePivotIOTalonFX(), new IntakeRollersIOTalonFX());
}
case DEVBOT -> {
  intake = new Intake(new IntakePivotIOTalonFX(), new IntakeRollersIOTalonFX());
}
case SIMBOT -> {
  intake = new Intake(new IntakePivotIOSim(), new IntakeRollersIOSim());
}

// Null fallback (for REPLAY or if not instantiated above)
if (intake == null) {
  intake = new Intake(new IntakePivotIO() {}, new IntakeRollersIO() {});
}
```

**Key rules:**
- COMPBOT/DEVBOT use `*IOTalonFX` implementations
- SIMBOT uses `*IOSim` implementations
- Null fallback uses anonymous no-op implementations (`new IO() {}`)
- The null fallback must appear AFTER the switch statement

## Command Factory Conventions
- `runOnce(...)` for instant state changes, named with `.withName("Subsystem.action")`
- `runEnd(action, cleanup)` for continuous actions that need cleanup on end
- `run(...)` for continuous actions without cleanup
- Combined commands should coordinate multiple mechanisms (e.g., deploy pivot + start rollers)

## @AutoLogOutput Usage
```java
@AutoLogOutput(key = "Subsystem/ValueName")
public double getMethod() { ... }
```
- Use for computed values derived from inputs (e.g., `getMeasuredAngleDeg()`)
- Use for boolean state queries (e.g., `isPivotAtGoal()`)
- Key path follows the `"Subsystem/..."` naming convention

## Common Gotchas
- `*IOInputsAutoLogged` classes won't resolve in IDE until after `./gradlew build` (annotation processor)
- Always run `./gradlew build` after creating/modifying IO interfaces to generate AutoLogged classes
- `FullSubsystem` auto-registers instances; the `Robot.java` loop calls `FullSubsystem.runAllPeriodicAfterScheduler()`
- Brake mode uses `Boolean` (boxed) for the `lastBrakeMode` field, initialized to `null`, so the first call always applies
- `Robot.showHardwareAlerts()` returns false in SIM mode and for the first 30 seconds of runtime