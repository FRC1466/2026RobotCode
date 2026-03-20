// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputMode;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.util.BatteryTracer;
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
  private static final LoggedTunableNumber shotDetectionCurrentAverageWindowSecs =
      new LoggedTunableNumber("Flywheel/ShotDetection/CurrentAverageWindowSecs");
  private static final LoggedTunableNumber shotDetectionMinVelocityRPS =
      new LoggedTunableNumber("Flywheel/ShotDetection/MinVelocityRPS");
  private static final LoggedTunableNumber shotDetectionCurrentSpikeThresholdAmps =
      new LoggedTunableNumber("Flywheel/ShotDetection/CurrentSpikeThresholdAmps");

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
    toleranceRotationsPerSec.initDefault(10);
    shotDetectionCurrentAverageWindowSecs.initDefault(2.0);
    shotDetectionMinVelocityRPS.initDefault(25.0);
    shotDetectionCurrentSpikeThresholdAmps.initDefault(5.0);
  }

  @Getter
  @Setter
  @AutoLogOutput(key = "Flywheel/ControlMode")
  private ControlMode controlMode = ControlMode.VOLTAGE;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput(key = "Flywheel/AtGoal")
  private boolean atGoal = false;

  // Shot detection tracks a rolling current baseline while the shooter is spun up. The average is
  // considered initialized after the first high-speed sample, and shotDetectedFromCurrentSpike
  // stores the previous cycle's detection state so we only record new rising edges.
  private LinearFilter shotDetectionCurrentAverageFilter;
  private double shotDetectionAverageCurrentAmps = 0.0;
  private boolean shotDetectionAverageInitialized = false;
  private boolean shotDetectedFromCurrentSpike = false;

  public Flywheel(FlywheelIO io) {
    this.io = io;
    shotDetectionCurrentAverageFilter = createShotDetectionCurrentAverageFilter();

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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (shotDetectionCurrentAverageWindowSecs.hasChanged(hashCode())) {
      resetShotDetection();
    }

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

    updateShotDetection();

    // In your periodic() method:
    double totalCurrent = 0.0;
    // For each motor, add its current (replace with your actual input fields)
    totalCurrent += inputs.supplyCurrentMasterAmps;
    totalCurrent += inputs.supplyCurrentFollowerAmps;
    // Repeat for all relevant motors in the subsystem
    BatteryTracer.addCurrent(
        "Flywheel", totalCurrent); // Replace "SubsystemName" with your subsystem

    // In your periodicAfterScheduler() method:
    BatteryTracer.publish("Flywheel"); // Replace "SubsystemName" with your subsystem

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

    switch (controlMode) {
      case VOLTAGE -> {
        outputs.mode = FlywheelIOOutputMode.VELOCITY_VOLTAGE;
      }
      case TORQUE_CURRENT -> {
        outputs.mode = FlywheelIOOutputMode.VELOCITY_TORQUE_CURRENT;
      }
      default -> {
        outputs.mode = FlywheelIOOutputMode.VELOCITY_VOLTAGE;
      }
    }
    outputs.velocityRotationsPerSec = velocityRotationsPerSec;
    outputs.feedforward =
        0; // Math.signum(velocityRotationsPerSec) * kS + velocityRotationsPerSec * kV;
    Logger.recordOutput("Flywheel/Setpoint", velocityRotationsPerSec);
  }

  /** Stops the flywheel. */
  public void stop() {
    outputs.mode = FlywheelIOOutputMode.COAST;
    outputs.velocityRotationsPerSec = 0.0;
    atGoal = false;
    resetShotDetection();
  }

  /** Returns the current velocity in rotations per second. */
  public double getVelocity() {
    return inputs.velocityRotationsPerSec;
  }

  public Command runTrackTargetCommand() {
    return runEnd(
        () -> runVelocity(ShotCalculator.getInstance().getParameters().flywheelSpeedRPS()),
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

  /** Converts a current-average window in seconds to the moving-average sample count. */
  static int getShotDetectionCurrentAverageWindowSamples(double windowSecs) {
    return Math.max(1, (int) Math.round(windowSecs / Constants.loopPeriodSecs));
  }

  /**
   * Returns whether a shot-like event has been detected from a current spike while the flywheel is
   * up to speed.
   */
  static boolean shouldRecordShotFromCurrentSpike(
      double currentAmps,
      double averageCurrentAmps,
      double velocityRotationsPerSec,
      double minVelocityRotationsPerSec,
      double currentSpikeThresholdAmps) {
    return Math.abs(velocityRotationsPerSec) >= minVelocityRotationsPerSec
        && currentAmps - averageCurrentAmps >= currentSpikeThresholdAmps;
  }

  private static LinearFilter createShotDetectionCurrentAverageFilter() {
    return LinearFilter.movingAverage(
        getShotDetectionCurrentAverageWindowSamples(shotDetectionCurrentAverageWindowSecs.get()));
  }

  /** Resets the current-spike detector when the flywheel stops or its averaging window changes. */
  private void resetShotDetection() {
    shotDetectionCurrentAverageFilter = createShotDetectionCurrentAverageFilter();
    shotDetectionAverageCurrentAmps = 0.0;
    shotDetectionAverageInitialized = false;
    shotDetectedFromCurrentSpike = false;
  }

  private void updateShotDetection() {
    boolean shotDetectionModeActive =
        outputs.mode == FlywheelIOOutputMode.VELOCITY_VOLTAGE
            || outputs.mode == FlywheelIOOutputMode.VELOCITY_TORQUE_CURRENT;
    boolean flywheelSpunUp =
        shotDetectionModeActive
            && Math.abs(inputs.velocityRotationsPerSec) >= shotDetectionMinVelocityRPS.get();
    double currentAmps = 0.0;
    double averageCurrentAmps = 0.0;

    if (flywheelSpunUp) {
      currentAmps =
          Math.abs(inputs.supplyCurrentMasterAmps) + Math.abs(inputs.supplyCurrentFollowerAmps);
      // Compare against the moving average from prior spun-up cycles before folding in the latest
      // current sample so a new shot is recorded as soon as the flywheel load jumps.
      averageCurrentAmps = shotDetectionAverageCurrentAmps;
      boolean currentSpikeDetected =
          shotDetectionAverageInitialized
              && shouldRecordShotFromCurrentSpike(
                  currentAmps,
                  averageCurrentAmps,
                  inputs.velocityRotationsPerSec,
                  shotDetectionMinVelocityRPS.get(),
                  shotDetectionCurrentSpikeThresholdAmps.get());
      boolean shotWasAlreadyDetected = shotDetectedFromCurrentSpike;
      shotDetectedFromCurrentSpike = currentSpikeDetected;
      if (currentSpikeDetected && !shotWasAlreadyDetected) {
        ShotCalculator.getInstance().recordShot();
      }
      shotDetectionAverageCurrentAmps = shotDetectionCurrentAverageFilter.calculate(currentAmps);
      shotDetectionAverageInitialized = true;
    } else {
      resetShotDetection();
    }

    Logger.recordOutput("Flywheel/ShotDetection/CurrentAmps", currentAmps);
    Logger.recordOutput("Flywheel/ShotDetection/AverageCurrentAmps", averageCurrentAmps);
    Logger.recordOutput(
        "Flywheel/ShotDetection/CurrentSpikeDetected", shotDetectedFromCurrentSpike);
  }
}
