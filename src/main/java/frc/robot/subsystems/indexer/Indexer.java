// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOOutputs;
import frc.robot.util.BatteryTracer;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Indexer extends FullSubsystem {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final IndexerIOOutputs outputs = new IndexerIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;

  @Setter private BooleanSupplier coastOverride = () -> true;

  private Boolean lastBrakeMode = null;

  // Stall detection
  @Getter private boolean stalled = false;

  private boolean reversing = false;
  private double reverseStartTime = 0.0;

  private static final LoggedTunableNumber runVolts =
      new LoggedTunableNumber("Indexer/RunVolts", 6.0);
  private static final LoggedTunableNumber reverseVolts =
      new LoggedTunableNumber("Indexer/ReverseVolts", -4.0);
  private static final LoggedTunableNumber stallCurrentThreshold =
      new LoggedTunableNumber("Indexer/StallCurrentThreshold", 90.0);
  private static final LoggedTunableNumber stallVelocityThreshold =
      new LoggedTunableNumber("Indexer/StallVelocityThreshold", 0.5);
  private static final LoggedTunableNumber stallDebounceTime =
      new LoggedTunableNumber("Indexer/StallDebounceSecs", 0.15);
  private static final LoggedTunableNumber reverseDuration =
      new LoggedTunableNumber("Indexer/ReverseDurationSecs", 0.25);

  private Debouncer stallDebouncer;

  @Getter private boolean running = false;

  static {
    switch (Constants.getMode()) {
      case REAL, REPLAY -> {
        runVolts.initDefault(6.0);
        reverseVolts.initDefault(-4.0);
        stallCurrentThreshold.initDefault(90.0);
        stallVelocityThreshold.initDefault(0.5);
        stallDebounceTime.initDefault(0.15);
        reverseDuration.initDefault(0.25);
      }
      case SIM -> {
        runVolts.initDefault(6.0);
        reverseVolts.initDefault(-4.0);
        stallCurrentThreshold.initDefault(90.0);
        stallVelocityThreshold.initDefault(0.5);
        stallDebounceTime.initDefault(0.15);
        reverseDuration.initDefault(0.25);
      }
    }
  }

  public Indexer(IndexerIO io) {
    this.io = io;

    disconnected = new Alert("Indexer motor disconnected!", Alert.AlertType.kWarning);

    stallDebouncer = new Debouncer(stallDebounceTime.get(), Debouncer.DebounceType.kRising);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    if (stallDebounceTime.hasChanged(hashCode())) {
      stallDebouncer = new Debouncer(stallDebounceTime.get(), Debouncer.DebounceType.kRising);
    }

    if (DriverStation.isDisabled()) {
      stop();
    }

    // Brake/coast mode
    boolean shouldBrake = !(DriverStation.isDisabled() || coastOverride.getAsBoolean());
    if (lastBrakeMode == null || lastBrakeMode != shouldBrake) {
      io.setBrakeMode(shouldBrake);
      lastBrakeMode = shouldBrake;
    }

    // Stall detection & auto-reverse logic
    if (running && !DriverStation.isDisabled()) {
      boolean rawStall =
          Math.abs(inputs.torqueCurrentAmps) > stallCurrentThreshold.get()
              && Math.abs(inputs.velocityRps) < stallVelocityThreshold.get();

      stalled = stallDebouncer.calculate(rawStall);

      if (reversing) {
        // Currently reversing to clear a jam
        outputs.appliedVolts = reverseVolts.get();
        if (Timer.getTimestamp() - reverseStartTime > reverseDuration.get()) {
          reversing = false;
        }
      } else if (stalled) {
        // Stall detected, begin reversing
        reversing = true;
        reverseStartTime = Timer.getTimestamp();
        outputs.appliedVolts = reverseVolts.get();
      } else {
        // Normal forward operation
        outputs.appliedVolts = runVolts.get();
      }
    }

    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));

    Logger.recordOutput("Indexer/Running", running);
    Logger.recordOutput("Indexer/Stalled", stalled);
    Logger.recordOutput("Indexer/Reversing", reversing);
    Logger.recordOutput("Indexer/AppliedVolts", outputs.appliedVolts);

    // In your periodic() method:
    double totalCurrent = 0.0;
    // For each motor, add its current (replace with your actual input fields)
    totalCurrent += inputs.supplyCurrentAmps;
    // Repeat for all relevant motors in the subsystem
    BatteryTracer.addCurrent(
        "Indexer", totalCurrent); // Replace "SubsystemName" with your subsystem

    // In your periodicAfterScheduler() method:
    BatteryTracer.publish("Indexer"); // Replace "SubsystemName" with your subsystem

    LoggedTracer.record("Indexer/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
    LoggedTracer.record("Indexer/AfterScheduler");
  }

  /** Run the indexer forward at the default voltage. */
  public void run() {
    running = true;
  }

  public void runBackwards() {
    reversing = true;
    running = true;
  }

  /** Run the indexer at a custom voltage (bypasses stall detection). */
  public void runVolts(double volts) {
    running = true;
    reversing = false;
    outputs.appliedVolts = volts;
  }

  /** Stop the indexer. */
  public void stop() {
    running = false;
    reversing = false;
    stalled = false;
    outputs.appliedVolts = 0.0;
  }

  /** Command to run the indexer (with automatic stall detection). */
  public Command runCommand() {
    return runEnd(this::run, this::stop).withName("Indexer.run");
  }

  /** Command to run the indexer backwards (with automatic stall detection). */
  public Command runBackwardsCommand() {
    return runEnd(this::runBackwards, this::stop).withName("Indexer.runBackwards");
  }

  /** Command to stop the indexer. */
  public Command stopCommand() {
    return runOnce(this::stop).withName("Indexer.stop");
  }
}
