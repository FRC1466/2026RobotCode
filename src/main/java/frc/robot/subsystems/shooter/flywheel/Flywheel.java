// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends FullSubsystem {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;
  private final SysIdRoutine sysId;

  @Setter private BooleanSupplier coastOverride = () -> false;

  public static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.19);
  public static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 0.11);
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 0.3);
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD", 0.0);

  public Flywheel(FlywheelIO io) {
    this.io = io;

    disconnected = new Alert("Flywheel motor disconnected!", Alert.AlertType.kWarning);

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())) {
      outputs.kP = kP.get();
      outputs.kD = kD.get();
      outputs.kS = kS.get();
      outputs.kV = kV.get();
    }

    if (DriverStation.isDisabled()) {
      stop();
      if (coastOverride.getAsBoolean()) {
        outputs.coast = true;
      }
    }

    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRps) {
    outputs.controlMode = FlywheelIO.FlywheelIOOutputs.ControlMode.VELOCITY;
    outputs.coast = false;
    outputs.velocityRps = velocityRps;
    outputs.feedForward = 0.0;

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/Setpoint", outputs.velocityRps);
  }

  /** Stops the flywheel. */
  public void stop() {
    outputs.controlMode = FlywheelIO.FlywheelIOOutputs.ControlMode.VOLTAGE;
    outputs.appliedVolts = 0.0;
    outputs.velocityRps = 0.0;
    outputs.coast = true;
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    outputs.controlMode = FlywheelIO.FlywheelIOOutputs.ControlMode.VOLTAGE;
    outputs.appliedVolts = volts;
    outputs.coast = false;
  }

  /** Returns the current velocity in RPS. */
  public double getVelocity() {
    return inputs.velocityRps;
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

  public Command runSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command runSysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
