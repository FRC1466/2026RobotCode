// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.intake.pivot.IntakePivotIO.IntakePivotIOOutputMode;
import frc.robot.subsystems.intake.pivot.IntakePivotIO.IntakePivotIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends FullSubsystem {
  private static final double stowAngleDeg = 0.1;
  private static final double deployAngleDeg = 110.0;
  private static final double minAngleDeg = 0.0;
  private static final double maxAngleDeg = 120.0;

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("IntakePivot/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("IntakePivot/kD");
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("IntakePivot/ToleranceDeg");

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

  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();
  private final IntakePivotIOOutputs outputs = new IntakePivotIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Intake pivot motor disconnected!", Alert.AlertType.kWarning);

  @Setter private BooleanSupplier coastOverride = () -> false;

  @Setter private double goalAngleDeg = stowAngleDeg;

  private Boolean lastBrakeMode = null;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    motorDisconnectedAlert.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.motorConnected));

    boolean shouldBrake = !coastOverride.getAsBoolean();
    if (lastBrakeMode == null || lastBrakeMode != shouldBrake) {
      io.setBrakeMode(shouldBrake);
      lastBrakeMode = shouldBrake;
    }

    outputs.kP = kP.get();
    outputs.kD = kD.get();
  }

  @Override
  public void periodicAfterScheduler() {
    if (DriverStation.isEnabled()) {
      double clampedGoalDeg = MathUtil.clamp(goalAngleDeg, minAngleDeg, maxAngleDeg);
      outputs.positionRad = Units.degreesToRadians(clampedGoalDeg);
      outputs.mode = IntakePivotIOOutputMode.CLOSED_LOOP;
      outputs.volts = 0.0;

      Logger.recordOutput("IntakePivot/GoalPositionDeg", clampedGoalDeg);
    } else {
      outputs.mode = IntakePivotIOOutputMode.OPEN_LOOP;
      outputs.volts = 0.0;
    }

    io.applyOutputs(outputs);
  }

  @AutoLogOutput(key = "IntakePivot/MeasuredAngleDeg")
  public double getMeasuredAngleDeg() {
    return Units.radiansToDegrees(inputs.positionRads);
  }

  @AutoLogOutput(key = "IntakePivot/AtGoal")
  public boolean isAtGoal() {
    return DriverStation.isEnabled()
        && Math.abs(getMeasuredAngleDeg() - goalAngleDeg) <= toleranceDeg.get();
  }

  /** Deploy the intake to the ground pickup position. */
  public void deploy() {
    setGoalAngleDeg(deployAngleDeg);
  }

  /** Stow the intake to the retracted position. */
  public void stow() {
    setGoalAngleDeg(stowAngleDeg);
  }

  /** Command to move the pivot to a fixed angle. */
  public Command runFixedCommand(DoubleSupplier angleDeg) {
    return run(() -> setGoalAngleDeg(angleDeg.getAsDouble()));
  }

  /** Command to deploy the intake. */
  public Command deployCommand() {
    return runOnce(this::deploy).withName("IntakePivot.deploy");
  }

  /** Command to stow the intake. */
  public Command stowCommand() {
    return runOnce(this::stow).withName("IntakePivot.stow");
  }

  /** Command to run open-loop at the specified voltage. */
  public Command runVolts(DoubleSupplier volts) {
    return run(
        () -> {
          outputs.mode = IntakePivotIOOutputMode.OPEN_LOOP;
          outputs.volts = volts.getAsDouble();
        });
  }
}
