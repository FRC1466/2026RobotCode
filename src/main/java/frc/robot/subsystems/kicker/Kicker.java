// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.kicker;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.kicker.KickerIO.KickerIOOutputs;
import frc.robot.util.BatteryTracer;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Kicker extends FullSubsystem {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
  private final KickerIOOutputs outputs = new KickerIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;

  @Setter private BooleanSupplier coastOverride = () -> true;

  private Boolean lastBrakeMode = null;

  @Getter private boolean running = false;

  private static final LoggedTunableNumber runVolts =
      new LoggedTunableNumber("Kicker/RunVolts", 8.0);

  public Kicker(KickerIO io) {
    this.io = io;

    disconnected = new Alert("Kicker motor disconnected!", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);

    if (DriverStation.isDisabled()) {
      stop();
    }

    // Brake/coast mode
    boolean shouldBrake = !(DriverStation.isDisabled() || coastOverride.getAsBoolean());
    if (lastBrakeMode == null || lastBrakeMode != shouldBrake) {
      io.setBrakeMode(shouldBrake);
      lastBrakeMode = shouldBrake;
    }

    // Apply voltage when running
    if (running && !DriverStation.isDisabled()) {
      outputs.appliedVolts = runVolts.get();
    }

    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));

    Logger.recordOutput("Kicker/Running", running);
    Logger.recordOutput("Kicker/AppliedVolts", outputs.appliedVolts);

    // In your periodic() method:
    double totalCurrent = 0.0;
    // For each motor, add its current (replace with your actual input fields)
    totalCurrent += inputs.supplyCurrentAmps;
    // Repeat for all relevant motors in the subsystem
    BatteryTracer.addCurrent("Kicker", totalCurrent); // Replace "SubsystemName" with your subsystem

    // In your periodicAfterScheduler() method:
    BatteryTracer.publish("Kicker"); // Replace "SubsystemName" with your subsystem

    LoggedTracer.record("Kicker/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
    LoggedTracer.record("Kicker/AfterScheduler");
  }

  /** Run the kicker forward at the default voltage. */
  public void run() {
    running = true;
  }

  /** Stop the kicker. */
  public void stop() {
    running = false;
    outputs.appliedVolts = 0.0;
  }

  /** Command to run the kicker. */
  public Command runCommand() {
    return runEnd(this::run, this::stop).withName("Kicker.run");
  }

  /** Command to stop the kicker. */
  public Command stopCommand() {
    return runOnce(this::stop).withName("Kicker.stop");
  }
}
