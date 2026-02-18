// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputMode;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends FullSubsystem {
  public enum ControlMode {
    VOLTAGE,
    TORQUE_CURRENT
  }

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer motorFollowerConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;
  private final Alert followerDisconnected;

  private static final LoggedTunableNumber voltageKP =
      new LoggedTunableNumber("Flywheel/Voltage/kP");
  private static final LoggedTunableNumber voltageKD =
      new LoggedTunableNumber("Flywheel/Voltage/kD");
  private static final LoggedTunableNumber voltageKS =
      new LoggedTunableNumber("Flywheel/Voltage/kS");
  private static final LoggedTunableNumber voltageKV =
      new LoggedTunableNumber("Flywheel/Voltage/kV");

  private static final LoggedTunableNumber torqueCurrentKP =
      new LoggedTunableNumber("Flywheel/TorqueCurrent/kP");
  private static final LoggedTunableNumber torqueCurrentKD =
      new LoggedTunableNumber("Flywheel/TorqueCurrent/kD");
  private static final LoggedTunableNumber torqueCurrentKS =
      new LoggedTunableNumber("Flywheel/TorqueCurrent/kS");
  private static final LoggedTunableNumber torqueCurrentKV =
      new LoggedTunableNumber("Flywheel/TorqueCurrent/kV");

  static {
    switch (Constants.getMode()) {
      case REAL, REPLAY -> {
        voltageKP.initDefault(0.1);
        voltageKD.initDefault(0.0);
        voltageKS.initDefault(0.23013);
        voltageKV.initDefault(0.12021);

        torqueCurrentKP.initDefault(0.1);
        torqueCurrentKD.initDefault(0.0);
        torqueCurrentKS.initDefault(0.0);
        torqueCurrentKV.initDefault(0.0);
      }
      case SIM -> {
        voltageKP.initDefault(0.1);
        voltageKD.initDefault(0.0);
        voltageKS.initDefault(0.23013);
        voltageKV.initDefault(0.12021);
      }
    }
  }

  @Getter
  @Setter
  @AutoLogOutput(key = "Flywheel/ControlMode")
  private ControlMode controlMode = ControlMode.VOLTAGE;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput(key = "Flywheel/AtGoal")
  private boolean atGoal = false;

  public Flywheel(FlywheelIO io) {
    this.io = io;

    disconnected = new Alert("Flywheel motor disconnected!", Alert.AlertType.kWarning);
    followerDisconnected =
        new Alert("Flywheel follower motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    outputs.voltageKP = voltageKP.get();
    outputs.voltageKD = voltageKD.get();
    outputs.voltageKS = voltageKS.get();
    outputs.voltageKV = voltageKV.get();

    outputs.torqueCurrentKP = torqueCurrentKP.get();
    outputs.torqueCurrentKD = torqueCurrentKD.get();
    outputs.torqueCurrentKS = torqueCurrentKS.get();
    outputs.torqueCurrentKV = torqueCurrentKV.get();

    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connectedMaster));
    followerDisconnected.set(
        Robot.showHardwareAlerts()
            && !motorFollowerConnectedDebouncer.calculate(inputs.connectedFollower));

    LoggedTracer.record("Flywheel/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Flywheel/Mode", outputs.mode);
    io.applyOutputs(outputs);

    LoggedTracer.record("Flywheel/AfterScheduler");
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRadsPerSec) {
    double kS;
    double kV;
    switch (controlMode) {
      case VOLTAGE -> {
        outputs.mode = FlywheelIOOutputMode.VELOCITY_VOLTAGE;
        kS = voltageKS.get();
        kV = voltageKV.get();
      }
      case TORQUE_CURRENT -> {
        outputs.mode = FlywheelIOOutputMode.VELOCITY_TORQUE_CURRENT;
        kS = torqueCurrentKS.get();
        kV = torqueCurrentKV.get();
      }
      default -> {
        outputs.mode = FlywheelIOOutputMode.VELOCITY_VOLTAGE;
        kS = voltageKS.get();
        kV = voltageKV.get();
      }
    }
    outputs.velocityRadsPerSec = velocityRadsPerSec;
    outputs.feedforward = Math.signum(velocityRadsPerSec) * kS + velocityRadsPerSec * kV;
    Logger.recordOutput("Flywheel/Setpoint", velocityRadsPerSec);
  }

  /** Stops the flywheel. */
  public void stop() {
    outputs.mode = FlywheelIOOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
    atGoal = false;
  }

  /** Returns the current velocity in RPM. */
  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public Command runTrackTargetCommand() {
    return runEnd(
        () -> runVelocity(ShotCalculator.getInstance().getParameters().flywheelSpeed()),
        this::stop);
  }

  public Command runFixedCommand(DoubleSupplier velocity) {
    return runEnd(() -> runVelocity(velocity.getAsDouble()), this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
