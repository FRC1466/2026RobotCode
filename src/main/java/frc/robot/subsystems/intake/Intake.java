// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.pivot.IntakePivotIO;
import frc.robot.subsystems.intake.pivot.IntakePivotIO.IntakePivotIOOutputs;
import frc.robot.subsystems.intake.pivot.IntakePivotIOInputsAutoLogged;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO.IntakeRollersIOOutputs;
import frc.robot.subsystems.intake.rollers.IntakeRollersIOInputsAutoLogged;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends FullSubsystem {
  private static final double stowAngleDeg = 0.1;
  private static final double deployAngleDeg = 105.0;
  private static final double minAngleDeg = 0.0;
  private static final double maxAngleDeg = 110.0;

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("IntakePivot/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("IntakePivot/kD");
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("IntakePivot/ToleranceDeg");
  private static final LoggedTunableNumber runVolts =
      new LoggedTunableNumber("IntakeRollers/RunVolts", 10.0);

  static {
    switch (Constants.getMode()) {
      case REAL, REPLAY -> {
        kP.initDefault(100);
        kD.initDefault(0);
      }
      case SIM -> {
        kP.initDefault(2.0);
        kD.initDefault(0.35);
      }
    }
    toleranceDeg.initDefault(2.0);
  }

  private final IntakePivotIO pivotIO;
  private final IntakeRollersIO rollersIO;

  private final IntakePivotIOInputsAutoLogged pivotInputs = new IntakePivotIOInputsAutoLogged();
  private final IntakePivotIOOutputs pivotOutputs = new IntakePivotIOOutputs();

  private final IntakeRollersIOInputsAutoLogged rollersInputs =
      new IntakeRollersIOInputsAutoLogged();
  private final IntakeRollersIOOutputs rollersOutputs = new IntakeRollersIOOutputs();

  private final Debouncer pivotMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert pivotMotorDisconnectedAlert =
      new Alert("Intake pivot motor disconnected!", Alert.AlertType.kWarning);

  private final Debouncer rollersMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert rollersDisconnectedAlert =
      new Alert("Intake rollers motor disconnected!", Alert.AlertType.kWarning);

  @Setter private BooleanSupplier pivotCoastOverride = () -> false;
  @Setter private BooleanSupplier rollersCoastOverride = () -> true;

  @Setter private double goalAngleDeg = stowAngleDeg;
  private Boolean lastPivotBrakeMode = null;

  private Boolean lastRollersBrakeMode = null;
  @Getter private boolean running = false;

  public Intake(IntakePivotIO pivotIO, IntakeRollersIO rollersIO) {
    this.pivotIO = pivotIO;
    this.rollersIO = rollersIO;
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("IntakePivot", pivotInputs);

    rollersIO.updateInputs(rollersInputs);
    Logger.processInputs("IntakeRollers", rollersInputs);

    pivotMotorDisconnectedAlert.set(
        Robot.showHardwareAlerts()
            && !pivotMotorConnectedDebouncer.calculate(pivotInputs.motorConnected));

    boolean pivotShouldBrake = !pivotCoastOverride.getAsBoolean();
    if (lastPivotBrakeMode == null || lastPivotBrakeMode != pivotShouldBrake) {
      pivotIO.setBrakeMode(pivotShouldBrake);
      lastPivotBrakeMode = pivotShouldBrake;
    }

    boolean rollersShouldBrake =
        !(DriverStation.isDisabled() || rollersCoastOverride.getAsBoolean());
    if (lastRollersBrakeMode == null || lastRollersBrakeMode != rollersShouldBrake) {
      rollersIO.setBrakeMode(rollersShouldBrake);
      lastRollersBrakeMode = rollersShouldBrake;
    }

    if (DriverStation.isDisabled()) {
      running = false;
    }

    if (running) {
      rollersOutputs.appliedVolts = runVolts.get();
    }

    rollersDisconnectedAlert.set(
        Robot.showHardwareAlerts()
            && !rollersMotorConnectedDebouncer.calculate(rollersInputs.connected));

    pivotOutputs.kP = kP.get();
    pivotOutputs.kD = kD.get();

    Logger.recordOutput("IntakeRollers/Running", running);
    Logger.recordOutput("IntakeRollers/AppliedVolts", rollersOutputs.appliedVolts);
  }

  @Override
  public void periodicAfterScheduler() {
    if (DriverStation.isEnabled()) {
      double clampedGoalDeg = MathUtil.clamp(goalAngleDeg, minAngleDeg, maxAngleDeg);
      pivotOutputs.positionRad = Units.degreesToRadians(clampedGoalDeg);
      pivotOutputs.mode = IntakePivotIO.IntakePivotIOOutputMode.CLOSED_LOOP;
      pivotOutputs.volts = 0.0;
      Logger.recordOutput("IntakePivot/GoalPositionDeg", clampedGoalDeg);
    } else {
      pivotOutputs.mode = IntakePivotIO.IntakePivotIOOutputMode.OPEN_LOOP;
      pivotOutputs.volts = 0.0;
    }

    pivotIO.applyOutputs(pivotOutputs);
    rollersIO.applyOutputs(rollersOutputs);
  }

  // Pivot API

  @AutoLogOutput(key = "IntakePivot/MeasuredAngleDeg")
  public double getMeasuredAngleDeg() {
    return Units.radiansToDegrees(pivotInputs.positionRads);
  }

  @AutoLogOutput(key = "IntakePivot/AtGoal")
  public boolean isAtGoal() {
    return DriverStation.isEnabled()
        && Math.abs(getMeasuredAngleDeg() - goalAngleDeg) <= toleranceDeg.get();
  }

  public void deploy() {
    setGoalAngleDeg(deployAngleDeg);
  }

  public void stow() {
    setGoalAngleDeg(stowAngleDeg);
  }

  /** Command to move the pivot to a fixed angle. */
  public Command runFixedCommand(DoubleSupplier angleDeg) {
    return run(() -> setGoalAngleDeg(angleDeg.getAsDouble()));
  }

  public Command deployCommand() {
    return runOnce(this::deploy).withName("IntakePivot.deploy");
  }

  public Command stowCommand() {
    return runOnce(this::stow).withName("IntakePivot.stow");
  }

  /** Run pivot open-loop at the specified voltage. */
  public Command runVolts(DoubleSupplier volts) {
    return run(
        () -> {
          pivotOutputs.mode = IntakePivotIO.IntakePivotIOOutputMode.OPEN_LOOP;
          pivotOutputs.volts = volts.getAsDouble();
        });
  }

  // Rollers API

  public void run() {
    running = true;
  }

  public void runVolts(double volts) {
    running = true;
    rollersOutputs.appliedVolts = volts;
  }

  public void stop() {
    running = false;
    rollersOutputs.appliedVolts = 0.0;
  }

  public Command runCommand() {
    return runEnd(this::run, this::stop).withName("IntakeRollers.run");
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("IntakeRollers.stop");
  }

  // Combined commands

  /** Deploy the pivot and run the rollers; stows and stops on end. */
  public Command intakeCommand() {
    return runEnd(
            () -> {
              deploy();
              run();
            },
            () -> {
              stow();
              stop();
            })
        .withName("Intake.intake");
  }

  /** Deploy the pivot and run the rollers without stopping/stowing on end. */
  public Command intakeNoStopCommand() {
    return runOnce(
            () -> {
              deploy();
              run();
            })
        .withName("Intake.intakeNoStop");
  }

  /** Stow the pivot and stop the rollers. */
  public Command stowAndStopCommand() {
    return runOnce(
            () -> {
              stow();
              stop();
            })
        .withName("Intake.stowAndStop");
  }
}
