// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
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

  /**
   * If true, force coast. Default to coast for safety and to reduce drag when the robot is
   * disabled.
   */
  @Setter private BooleanSupplier coastOverride = () -> true;

  @Getter @Setter private boolean useInternalBangBang = true;

  private Debouncer atGoalDebouncer;
  private Debouncer flywheelToleranceDebouncer;

  private boolean wasWithinTolerance = false;
  private long shotCount = 0;
  private boolean atGoal = false;
  private Boolean lastBrakeMode = null;

  private static final LoggedTunableNumber flywheelTolerance =
      new LoggedTunableNumber("Flywheel/Tolerance", 1);
  private static final LoggedTunableNumber flywheelToleranceDebounce =
      new LoggedTunableNumber("Flywheel/ToleranceDebounce", 0.025);
  private static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Flywheel/AtGoalDebounce", 0.2);

  public static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS");
  public static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV");
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP");
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD");

  static {
    switch (Constants.getMode()) {
      case REAL, REPLAY -> {
        kS.initDefault(0.19);
        kV.initDefault(0.11);
        kP.initDefault(0.3);
        kD.initDefault(0.0);
      }
      case SIM -> {
        kS.initDefault(0.0);
        kV.initDefault(0.0);
        kP.initDefault(12.0);
        kD.initDefault(0.0);
      }
    }
  }

  public Flywheel(FlywheelIO io) {
    this.io = io;

    disconnected = new Alert("Flywheel motor disconnected!", Alert.AlertType.kWarning);

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));

    atGoalDebouncer = new Debouncer(atGoalDebounce.get(), Debouncer.DebounceType.kFalling);
    flywheelToleranceDebouncer =
        new Debouncer(flywheelToleranceDebounce.get(), Debouncer.DebounceType.kFalling);
  }

  public boolean isAtGoal() {
    return atGoal;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (flywheelToleranceDebounce.hasChanged(hashCode())) {
      flywheelToleranceDebouncer =
          new Debouncer(flywheelToleranceDebounce.get(), Debouncer.DebounceType.kFalling);
    }
    if (atGoalDebounce.hasChanged(hashCode())) {
      atGoalDebouncer = new Debouncer(atGoalDebounce.get(), Debouncer.DebounceType.kFalling);
    }

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
    }

    // Brake/coast is configured separately from the control mode.
    boolean shouldBrake = !(DriverStation.isDisabled() || coastOverride.getAsBoolean());
    if (lastBrakeMode == null || lastBrakeMode != shouldBrake) {
      io.setBrakeMode(shouldBrake);
      lastBrakeMode = shouldBrake;
    }

    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));

    Logger.recordOutput("Flywheel/AtGoal", atGoal);
    Logger.recordOutput("Flywheel/ShotCount", shotCount);
    Logger.recordOutput("Flywheel/ControlMode", outputs.controlMode);
    Logger.recordOutput("Flywheel/Setpoint", outputs.velocityRps);
    Logger.recordOutput("Flywheel/CurrentVelocity", inputs.velocityRps);
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRps) {
    outputs.velocityRps = velocityRps;
    outputs.feedForward = 0.0;

    // Calculate at-goal / shot-counting
    boolean inTolerance = Math.abs(inputs.velocityRps - velocityRps) <= flywheelTolerance.get();

    // De-bounce
    boolean isWithinTolerance = flywheelToleranceDebouncer.calculate(inTolerance);
    atGoal = atGoalDebouncer.calculate(inTolerance);

    // Shot counting (falling edge of isWithinTolerance/inTolerance effectively)
    if (!isWithinTolerance && wasWithinTolerance) {
      shotCount++;
    }
    wasWithinTolerance = isWithinTolerance;

    // Control Mode Selection
    if (useInternalBangBang) {
      if (isWithinTolerance) {
        // When within tolerance (stable), use Velocity control to hold speed smoothly
        outputs.controlMode = FlywheelIO.FlywheelIOOutputs.ControlMode.VELOCITY;
      } else {
        // When outside tolerance (spin up or recovery), use aggressive Bang-Bang
        outputs.controlMode = FlywheelIO.FlywheelIOOutputs.ControlMode.DUTY_CYCLE_BANG_BANG;
      }
    } else {
      outputs.controlMode = FlywheelIO.FlywheelIOOutputs.ControlMode.VELOCITY;
    }

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/Setpoint", outputs.velocityRps);
  }

  /** Stops the flywheel. */
  public void stop() {
    outputs.controlMode = FlywheelIO.FlywheelIOOutputs.ControlMode.VOLTAGE;
    outputs.appliedVolts = 0.0;
    outputs.velocityRps = 0.0;
    atGoal = false;
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    outputs.controlMode = FlywheelIO.FlywheelIOOutputs.ControlMode.VOLTAGE;
    outputs.appliedVolts = volts;
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
