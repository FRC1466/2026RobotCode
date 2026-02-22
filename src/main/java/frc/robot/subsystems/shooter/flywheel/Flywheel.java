// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  private final SysIdRoutine sysId;

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

  private static final LoggedTunableNumber toleranceRotationsPerSec =
      new LoggedTunableNumber("Flywheel/ToleranceRotationsPerSec");

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
    toleranceRotationsPerSec.initDefault(10.0 / (2 * Math.PI));
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

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  outputs.mode = FlywheelIOOutputMode.VOLTAGE;
                  outputs.velocityRotationsPerSec = 0.0;
                  outputs.feedforward = voltage.in(edu.wpi.first.units.Units.Volts);
                },
                null,
                this,
                "Flywheel"));
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

    if (edu.wpi.first.wpilibj.DriverStation.isDisabled()) {
      stop();
    }

    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connectedMaster));
    followerDisconnected.set(
        Robot.showHardwareAlerts()
            && !motorFollowerConnectedDebouncer.calculate(inputs.connectedFollower));

    if (outputs.mode != FlywheelIOOutputMode.COAST) {
      atGoal =
          Math.abs(inputs.velocityRotationsPerSec - outputs.velocityRotationsPerSec)
              <= toleranceRotationsPerSec.get();
    } else {
      atGoal = false;
    }

    LoggedTracer.record("Flywheel/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Flywheel/Mode", outputs.mode);
    io.applyOutputs(outputs);

    LoggedTracer.record("Flywheel/AfterScheduler");
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRotationsPerSec) {
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
    outputs.velocityRotationsPerSec = velocityRotationsPerSec;
    outputs.feedforward = Math.signum(velocityRotationsPerSec) * kS + velocityRotationsPerSec * kV;
    Logger.recordOutput("Flywheel/Setpoint", velocityRotationsPerSec);
  }

  /** Stops the flywheel. */
  public void stop() {
    outputs.mode = FlywheelIOOutputMode.COAST;
    outputs.velocityRotationsPerSec = 0.0;
    atGoal = false;
  }

  /** Returns the current velocity in rotations per second. */
  public double getVelocity() {
    return inputs.velocityRotationsPerSec;
  }

  public Command runTrackTargetCommand() {
    return runEnd(
        () -> runVelocity(ShotCalculator.getInstance().getParameters().flywheelSpeed()),
        this::stop);
  }

  public Command runFixedCommand(DoubleSupplier velocity) {
    return runEnd(() -> runVelocity(velocity.getAsDouble()), this::stop);
  }

  public Command runVolts(DoubleSupplier volts) {
    return runEnd(
        () -> {
          outputs.mode = FlywheelIOOutputMode.VOLTAGE;
          outputs.velocityRotationsPerSec = 0.0;
          outputs.feedforward = volts.getAsDouble();
        },
        this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
