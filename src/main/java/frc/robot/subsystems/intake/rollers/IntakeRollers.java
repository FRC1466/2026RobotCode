// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO.IntakeRollersIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends FullSubsystem {
  private final IntakeRollersIO io;
  private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();
  private final IntakeRollersIOOutputs outputs = new IntakeRollersIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;

  @Setter private BooleanSupplier coastOverride = () -> true;

  private Boolean lastBrakeMode = null;

  @Getter private boolean running = false;

  private static final LoggedTunableNumber runVolts =
      new LoggedTunableNumber("IntakeRollers/RunVolts", 10.0);

  public IntakeRollers(IntakeRollersIO io) {
    this.io = io;

    disconnected = new Alert("Intake rollers motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRollers", inputs);

    if (DriverStation.isDisabled()) {
      stop();
    }

    // Brake/coast mode
    boolean shouldBrake = !(DriverStation.isDisabled() || coastOverride.getAsBoolean());
    if (lastBrakeMode == null || lastBrakeMode != shouldBrake) {
      io.setBrakeMode(shouldBrake);
      lastBrakeMode = shouldBrake;
    }

    if (running && !DriverStation.isDisabled()) {
      outputs.appliedVolts = runVolts.get();
    }

    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));

    Logger.recordOutput("IntakeRollers/Running", running);
    Logger.recordOutput("IntakeRollers/AppliedVolts", outputs.appliedVolts);
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  /** Run the intake rollers forward at the default voltage. */
  public void run() {
    running = true;
  }

  /** Run the intake rollers at a specific voltage. */
  public void runVolts(double volts) {
    running = true;
    outputs.appliedVolts = volts;
  }

  /** Stop the intake rollers. */
  public void stop() {
    running = false;
    outputs.appliedVolts = 0.0;
  }

  /** Command to run the intake rollers. */
  public Command runCommand() {
    return runEnd(this::run, this::stop).withName("IntakeRollers.run");
  }

  /** Command to stop the intake rollers. */
  public Command stopCommand() {
    return runOnce(this::stop).withName("IntakeRollers.stop");
  }
}
