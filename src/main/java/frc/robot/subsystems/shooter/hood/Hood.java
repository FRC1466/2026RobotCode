// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputMode;
import frc.robot.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Hood extends FullSubsystem {
  public static final double minAngleDeg = 0.1;
  public static final double maxAngleDeg = 31.0;

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Hood/kV");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS");
  private static final LoggedTunableNumber cruiseVelocity =
      new LoggedTunableNumber("Hood/CruiseVelocity");
  private static final LoggedTunableNumber acceleration =
      new LoggedTunableNumber("Hood/Acceleration");
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/ToleranceDeg");

  static {
    switch (Constants.getMode()) {
      case REAL, REPLAY -> {
        kP.initDefault(1000);
        kD.initDefault(0);
        kV.initDefault(0);
        kS.initDefault(0);
        cruiseVelocity.initDefault(4);
        acceleration.initDefault(4);
      }
      case SIM -> {
        kP.initDefault(2.0);
        kD.initDefault(0.35);
        kV.initDefault(0);
        kS.initDefault(0);
        cruiseVelocity.initDefault(0);
        acceleration.initDefault(0);
      }
    }
    toleranceDeg.initDefault(1.0);
  }

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  // Mechanism2d visualization
  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.0, 1.0);
  private final LoggedMechanismRoot2d root = mechanism.getRoot("Hood", 0.5, 0.2);
  private final LoggedMechanismLigament2d hoodLigament =
      root.append(new LoggedMechanismLigament2d("Hood", 0.4, 0.0, 6, new Color8Bit(Color.kOrange)));
  private final LoggedMechanismLigament2d goalLigament =
      root.append(
          new LoggedMechanismLigament2d("HoodGoal", 0.4, 0.0, 2, new Color8Bit(Color.kGray)));

  // Connected debouncer
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Hood motor disconnected!", Alert.AlertType.kWarning);

  @Setter private BooleanSupplier coastOverride = () -> false;

  /** Hood goal angle in degrees. */
  @Setter private double goalAngleDeg = minAngleDeg;

  private Boolean lastBrakeMode = null;

  public Hood(HoodIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    motorDisconnectedAlert.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.motorConnected));

    // Brake/coast is configured separately from the control mode.
    // Default is brake unless overridden to coast.
    boolean shouldBrake = !coastOverride.getAsBoolean();
    if (lastBrakeMode == null || lastBrakeMode != shouldBrake) {
      io.setBrakeMode(shouldBrake);
      lastBrakeMode = shouldBrake;
    }

    // Update tunable numbers
    outputs.kP = kP.get();
    outputs.kD = kD.get();
    outputs.kV = kV.get();
    outputs.kS = kS.get();
    outputs.cruiseVelocity = cruiseVelocity.get();
    outputs.acceleration = acceleration.get();

    // Update mechanism visualization
    hoodLigament.setAngle(getMeasuredAngleDeg());
    goalLigament.setAngle(goalAngleDeg);
    Logger.recordOutput("Hood/Mechanism2d", mechanism);
    Logger.recordOutput("Hood/Mechanism3d", mechanism.generate3dMechanism().toArray(new Pose3d[0]));

    // Record cycle time
    LoggedTracer.record("Hood");
  }

  @Override
  public void periodicAfterScheduler() {
    if (DriverStation.isEnabled()) {
      double clampedGoalDeg = MathUtil.clamp(goalAngleDeg, minAngleDeg, maxAngleDeg);
      outputs.positionRotations = clampedGoalDeg / 360.0;
      outputs.mode = HoodIOOutputMode.CLOSED_LOOP;
      outputs.volts = 0.0;

      // Log state
      Logger.recordOutput("Hood/Profile/GoalPositionDeg", clampedGoalDeg);
    } else {
      outputs.mode = HoodIOOutputMode.OPEN_LOOP;
      outputs.volts = 0.0;
    }

    io.applyOutputs(outputs);
  }

  @AutoLogOutput(key = "Hood/MeasuredAngleDeg")
  public double getMeasuredAngleDeg() {
    return inputs.positionRotations * 360.0;
  }

  @AutoLogOutput(key = "Hood/AtGoal")
  public boolean isAtGoal() {
    return DriverStation.isEnabled()
        && Math.abs(getMeasuredAngleDeg() - goalAngleDeg) <= toleranceDeg.get();
  }

  public Command runTrackTargetCommand() {
    return run(
        () -> {
          var params = ShotCalculator.getInstance().getParameters();
          setGoalAngleDeg(params.hoodAngleDeg());
        });
  }

  public Command runFixedCommand(DoubleSupplier angle) {
    return run(() -> setGoalAngleDeg(angle.getAsDouble()));
  }

  /** Stows the hood to the minimum safe angle. */
  public void stow() {
    setGoalAngleDeg(minAngleDeg);
  }

  public Command runVolts(DoubleSupplier volts) {
    return run(
        () -> {
          outputs.mode = HoodIOOutputMode.OPEN_LOOP;
          outputs.volts = volts.getAsDouble();
        });
  }
}
