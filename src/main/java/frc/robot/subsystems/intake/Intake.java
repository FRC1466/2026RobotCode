// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.intake;

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
import frc.robot.subsystems.intake.pivot.IntakePivotIO;
import frc.robot.subsystems.intake.pivot.IntakePivotIO.IntakePivotIOOutputs;
import frc.robot.subsystems.intake.pivot.IntakePivotIOInputsAutoLogged;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO.IntakeRollersIOOutputs;
import frc.robot.subsystems.intake.rollers.IntakeRollersIOInputsAutoLogged;
import frc.robot.util.BatteryTracer;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Intake extends FullSubsystem {
  private static final LoggedTunableNumber stowAngleDeg =
      new LoggedTunableNumber("IntakePivot/StowAngleDeg", 8.0);
  private static final LoggedTunableNumber deployAngleDeg =
      new LoggedTunableNumber("IntakePivot/DeployAngleDeg", 75);
  private static final LoggedTunableNumber minAngleDeg =
      new LoggedTunableNumber("IntakePivot/MinAngleDeg", 0.0);
  private static final LoggedTunableNumber maxAngleDeg =
      new LoggedTunableNumber("IntakePivot/MaxAngleDeg", 85.0);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("IntakePivot/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("IntakePivot/kD");
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("IntakePivot/ToleranceDeg");
  private static final LoggedTunableNumber runVolts =
      new LoggedTunableNumber("IntakeRollers/RunVolts", 5.5);
  private static final LoggedTunableNumber homingRollerReverseVolts =
      new LoggedTunableNumber("Intake/Homing/RollerReverseVolts", -2.0);
  private static final LoggedTunableNumber homingPulseOnSec =
      new LoggedTunableNumber("Intake/Homing/PulseOnSec", 0.2);
  private static final LoggedTunableNumber homingPulseOffSec =
      new LoggedTunableNumber("Intake/Homing/PulseOffSec", 0.1);
  private static final LoggedTunableNumber homingDurationSec =
      new LoggedTunableNumber("Intake/Homing/DurationSec", 0.75);

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
    homingRollerReverseVolts.initDefault(-2.0);
    homingPulseOnSec.initDefault(0.2);
    homingPulseOffSec.initDefault(0.1);
    homingDurationSec.initDefault(0.75);
  }

  private final IntakePivotIO pivotIO;
  private final IntakeRollersIO rollersIO;

  private final IntakePivotIOInputsAutoLogged pivotInputs = new IntakePivotIOInputsAutoLogged();
  private final IntakePivotIOOutputs pivotOutputs = new IntakePivotIOOutputs();

  private final IntakeRollersIOInputsAutoLogged rollersInputs =
      new IntakeRollersIOInputsAutoLogged();
  private final IntakeRollersIOOutputs rollersOutputs = new IntakeRollersIOOutputs();

  // Mechanism2d visualization (4-bar linkage)
  // Ground link is the horizontal distance between the two frame pivots on the robot edge.
  // Crank (driven link) and rocker (follower link) rotate; coupler keeps rollers level.
  // At 0° motor angle the intake is stowed (pointing up); at deployAngleDeg it is deployed
  // (swung outward/down past the frame perimeter).
  private static final double groundLinkLength = 0.15; // m, offset between frame pivots
  private static final double crankLength = 0.35; // m, driven arm
  private static final double couplerLength = 0.15; // m, bar connecting crank tip to rocker tip
  private static final double rockerLength = 0.35; // m, follower arm

  private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.2, 1.2);
  // Pivots are near the front edge of the robot (high x).
  // Crank root is to the left of rocker root (ground link is horizontal).
  private final LoggedMechanismRoot2d crankRoot =
      mechanism.getRoot("CrankPivot", 0.9 - groundLinkLength, 0.2);
  private final LoggedMechanismLigament2d crankLigament =
      crankRoot.append(
          new LoggedMechanismLigament2d("Crank", crankLength, 90.0, 6, new Color8Bit(Color.kCyan)));
  private final LoggedMechanismLigament2d couplerLigament =
      crankLigament.append(
          new LoggedMechanismLigament2d(
              "Coupler", couplerLength, 0.0, 4, new Color8Bit(Color.kGreen)));
  // Rocker root is to the right of crank root
  private final LoggedMechanismRoot2d rockerRoot = mechanism.getRoot("RockerPivot", 0.9, 0.2);
  private final LoggedMechanismLigament2d rockerLigament =
      rockerRoot.append(
          new LoggedMechanismLigament2d(
              "Rocker", rockerLength, 90.0, 6, new Color8Bit(Color.kCyan)));
  // Goal overlay (thin lines)
  private final LoggedMechanismLigament2d crankGoalLigament =
      crankRoot.append(
          new LoggedMechanismLigament2d(
              "CrankGoal", crankLength, 90.0, 2, new Color8Bit(Color.kGray)));
  private final LoggedMechanismLigament2d rockerGoalLigament =
      rockerRoot.append(
          new LoggedMechanismLigament2d(
              "RockerGoal", rockerLength, 90.0, 2, new Color8Bit(Color.kGray)));

  private final Debouncer pivotMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert pivotMotorDisconnectedAlert =
      new Alert("Intake pivot motor disconnected!", Alert.AlertType.kWarning);

  private final Debouncer rollersMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert rollersDisconnectedAlert =
      new Alert("Intake rollers motor disconnected!", Alert.AlertType.kWarning);

  @Setter private BooleanSupplier pivotCoastOverride = () -> false;
  @Setter private BooleanSupplier rollersCoastOverride = () -> true;

  @Setter private double goalAngleDeg = stowAngleDeg.get();
  private Boolean lastPivotBrakeMode = null;

  private Boolean lastRollersBrakeMode = null;
  @Getter private boolean running = false;
  @Getter private boolean homed = false;
  private boolean zeroedThisCycle = false;
  private double lastZeroedPositionRotations = Double.NaN;

  public Intake(IntakePivotIO pivotIO, IntakeRollersIO rollersIO) {
    this.pivotIO = pivotIO;
    this.rollersIO = rollersIO;
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("IntakePivot", pivotInputs);

    rollersIO.updateInputs(rollersInputs);
    Logger.processInputs("IntakeRollers", rollersInputs);

    pivotMotorDisconnectedAlert.set(
        Robot.showHardwareAlerts()
            && !pivotMotorConnectedDebouncer.calculate(pivotInputs.motorConnected));

    boolean pivotShouldBrake = !pivotCoastOverride.getAsBoolean();
    if (lastPivotBrakeMode == null || lastPivotBrakeMode != pivotShouldBrake) {
      pivotIO.setBrakeMode(pivotShouldBrake);
      lastPivotBrakeMode = pivotShouldBrake;
    }

    boolean rollersShouldBrake =
        !(DriverStation.isDisabled() || rollersCoastOverride.getAsBoolean());
    if (lastRollersBrakeMode == null || lastRollersBrakeMode != rollersShouldBrake) {
      rollersIO.setBrakeMode(rollersShouldBrake);
      lastRollersBrakeMode = rollersShouldBrake;
    }

    if (DriverStation.isDisabled()) {
      running = false;
    }

    if (running) {
      rollersOutputs.appliedVolts = runVolts.get();
    }

    rollersDisconnectedAlert.set(
        Robot.showHardwareAlerts()
            && !rollersMotorConnectedDebouncer.calculate(rollersInputs.connected));

    pivotOutputs.kP = kP.get();
    pivotOutputs.kD = kD.get();

    // Update 4-bar mechanism visualization
    updateMechanism(getMeasuredAngleDeg(), crankLigament, rockerLigament, couplerLigament);
    updateMechanism(goalAngleDeg, crankGoalLigament, rockerGoalLigament, null);
    Logger.recordOutput("Intake/Mechanism2d", mechanism);
    Logger.recordOutput(
        "Intake/Mechanism3d", mechanism.generate3dMechanism().toArray(new Pose3d[0]));

    Logger.recordOutput("IntakeRollers/Running", running);
    Logger.recordOutput("IntakeRollers/AppliedVolts", rollersOutputs.appliedVolts);
    Logger.recordOutput("Intake/Homed", homed);
    Logger.recordOutput("Intake/HomingJustZeroed", zeroedThisCycle);
    Logger.recordOutput("IntakePivot/LastZeroedPositionRotations", lastZeroedPositionRotations);
    zeroedThisCycle = false;

    LoggedTracer.record("Intake/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    // Only set closed-loop if nothing else (runVolts) has taken control
    if (DriverStation.isEnabled()
        && pivotOutputs.mode != IntakePivotIO.IntakePivotIOOutputMode.OPEN_LOOP) {
      double clampedGoalDeg = MathUtil.clamp(goalAngleDeg, minAngleDeg.get(), maxAngleDeg.get());
      pivotOutputs.positionRotations = clampedGoalDeg / 360.0;
      pivotOutputs.mode = IntakePivotIO.IntakePivotIOOutputMode.CLOSED_LOOP;
      pivotOutputs.volts = 0.0;
      Logger.recordOutput("IntakePivot/GoalPositionDeg", clampedGoalDeg);
    } else if (!DriverStation.isEnabled()) {
      pivotOutputs.mode = IntakePivotIO.IntakePivotIOOutputMode.OPEN_LOOP;
      pivotOutputs.volts = 0.0;
    }

    pivotIO.applyOutputs(pivotOutputs);
    rollersIO.applyOutputs(rollersOutputs);

    // Reset mode for next cycle so commands must actively claim OPEN_LOOP
    if (DriverStation.isEnabled()) {
      pivotOutputs.mode = IntakePivotIO.IntakePivotIOOutputMode.CLOSED_LOOP;
    }

    // In your periodic() method:
    double totalCurrent = 0.0;
    // For each motor, add its current (replace with your actual input fields)
    totalCurrent += pivotInputs.supplyCurrentAmps;
    totalCurrent += rollersInputs.supplyCurrentAmps;
    // Repeat for all relevant motors in the subsystem
    BatteryTracer.addCurrent("Intake", totalCurrent); // Replace "SubsystemName" with your subsystem

    // In your periodicAfterScheduler() method:
    BatteryTracer.publish("Intake"); // Replace "SubsystemName" with your subsystem

    LoggedTracer.record("Intake/AfterScheduler");
  }

  // Pivot API

  @AutoLogOutput(key = "IntakePivot/MeasuredAngleDeg")
  public double getMeasuredAngleDeg() {
    return pivotInputs.positionRotations * 360.0;
  }

  @AutoLogOutput(key = "IntakePivot/AtGoal")
  public boolean isAtGoal() {
    return DriverStation.isEnabled()
        && Math.abs(getMeasuredAngleDeg() - goalAngleDeg) <= toleranceDeg.get();
  }

  @AutoLogOutput(key = "IntakePivot/IsDeployed")
  public boolean isDeployed() {
    return goalAngleDeg > stowAngleDeg.get();
  }

  public void deploy() {
    homed = false;
    setGoalAngleDeg(deployAngleDeg.get());
  }

  public void stow() {
    setGoalAngleDeg(stowAngleDeg.get());
  }

  /** Command to move the pivot to a fixed angle. */
  public Command runFixedCommand(DoubleSupplier angleDeg) {
    return run(() -> setGoalAngleDeg(angleDeg.getAsDouble()));
  }

  public Command deployCommand() {
    return runOnce(this::deploy).withName("IntakePivot.deploy");
  }

  public Command stowCommand() {
    return runOnce(this::stow).withName("IntakePivot.stow");
  }

  /** Run pivot open-loop at the specified voltage. */
  public Command runVolts(DoubleSupplier volts) {
    return run(
        () -> {
          pivotOutputs.mode = IntakePivotIO.IntakePivotIOOutputMode.OPEN_LOOP;
          pivotOutputs.volts = volts.getAsDouble();
        });
  }

  // Rollers API

  public void run() {
    running = true;
  }

  public void runVolts(double volts) {
    running = true;
    rollersOutputs.appliedVolts = volts;
  }

  public void runVoltsPivot(double volts) {
    running = false;
    pivotOutputs.mode = IntakePivotIO.IntakePivotIOOutputMode.OPEN_LOOP;
    pivotOutputs.volts = volts;
  }

  public void stop() {
    running = false;
    rollersOutputs.appliedVolts = 0.0;
  }

  public void markHomed() {
    homed = true;
  }

  public void clearHomed() {
    homed = false;
  }

  public Command runCommand() {
    return runEnd(this::run, this::stop).withName("IntakeRollers.run");
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("IntakeRollers.stop");
  }

  // Combined commands

  /** Deploy the pivot and run the rollers; stows and stops on end. */
  public Command intakeCommand() {
    return runEnd(
            () -> {
              deploy();
              run();
            },
            () -> {
              stow();
              stop();
            })
        .withName("Intake.intake");
  }

  /** Deploy the pivot and run the rollers without stopping/stowing on end. */
  public Command intakeNoStopCommand() {
    return runOnce(
            () -> {
              deploy();
              run();
            })
        .withName("Intake.intakeNoStop");
  }

  /** Stow the pivot and stop the rollers. */
  public Command stowAndStopCommand() {
    return runOnce(
            () -> {
              stow();
              stop();
            })
        .withName("Intake.stowAndStop");
  }

  /**
   * Stows the intake while pulsing the rollers in reverse to help clear and settle the mechanism.
   */
  public Command homeCommand() {
    return runEnd(
            () -> {
              stow();
              // Run the pivot at a negative voltage to push into the 0 position
              runVoltsPivot(-2.0); // Consider making this a tunable number if needed
            },
            () -> {
              stop();
              stow();
              lastZeroedPositionRotations = 0.0;
              pivotIO.resetPosition(lastZeroedPositionRotations);
              zeroedThisCycle = true;
              setGoalAngleDeg(stowAngleDeg.get());
              markHomed();
            })
        .beforeStarting(
            () -> {
              clearHomed();
              stop();
              stow();
            })
        .withTimeout(homingDurationSec.get())
        .ignoringDisable(true)
        .withName("Intake.home");
  }

  /**
   * Updates the 4-bar mechanism ligaments for a given motor angle.
   *
   * <p>The intake uses a parallelogram 4-bar linkage (crank == rocker length, ground == coupler
   * length). In a parallelogram, the rocker always mirrors the crank angle, keeping the coupler
   * (and the roller assembly attached to it) at a constant orientation.
   *
   * <p>Layout (side view, looking at the robot from the right):
   *
   * <ul>
   *   <li>Ground link (fixed, horizontal): from crank root (B, left) to rocker root (A, right).
   *   <li>Crank (driven): from B, length {@code crankLength}.
   *   <li>Rocker (follower): from A, length {@code rockerLength}, always parallel to crank.
   *   <li>Coupler: from crank tip (C) to rocker tip (D), always parallel to ground.
   * </ul>
   *
   * <p>Motor angle convention: 0° = stowed (arms straight up), increasing = clockwise rotation
   * (deploying outward). Display angle = {@code 90° − motorAngle}.
   */
  private void updateMechanism(
      double motorAngleDeg,
      LoggedMechanismLigament2d crank,
      LoggedMechanismLigament2d rocker,
      LoggedMechanismLigament2d coupler) {
    // Convert motor angle to Mechanism2d display angle (CCW from +X)
    // 0° motor = straight up = 90° display; increasing motor = CW = decreasing display
    double displayDeg = 90.0 - motorAngleDeg;

    // Parallelogram: rocker angle == crank angle
    crank.setAngle(displayDeg);
    rocker.setAngle(displayDeg);

    if (coupler != null) {
      // The coupler must close the loop from crank tip back to rocker tip.
      // In a parallelogram the coupler is always parallel to the ground link.
      // The ground link direction (B→A) in display coords is 0° (pointing right).
      // Coupler goes C→D which is the same direction as B→A (i.e. rightward).
      // Mechanism2d ligament angle is relative to its parent (crank), so:
      //   couplerRelative = couplerAbsolute − crankAbsolute = 0° − displayDeg
      coupler.setAngle(-displayDeg);
    }
  }
}
