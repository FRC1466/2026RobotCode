// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Choreographer extends SubsystemBase {

  public enum Goal {
    IDLE,
    INTAKE,
    SCORE_HUB,
    CLIMB_EXTEND,
    CLIMB_RETRACT
  }

  public enum State {
    IDLE,
    INTAKING,
    SPINNING_UP,
    WAITING_FOR_ALIGNMENT,
    SHOOTING,
    CLIMB_EXTENDING,
    CLIMB_RETRACTING
  }

  @Getter
  @AutoLogOutput(key = "Choreographer/Goal")
  private Goal currentGoal = Goal.IDLE;

  @Getter
  @AutoLogOutput(key = "Choreographer/State")
  private State currentState = State.IDLE;

  /** When false, Choreographer does not control any subsystems (for manual tuning). */
  @Getter
  @Setter
  @AutoLogOutput(key = "Choreographer/Enabled")
  private boolean enabled = true;

  private final Drive drive;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Indexer indexer;
  private final Kicker kicker;
  private final Intake intake;
  private final BooleanSupplier driveAlignedForHubShot;

  private ShootingParameters cachedShotParams = null;
  private boolean cachedDriveAlignedForHubShot = false;
  private final LoggedNetworkBoolean useHubShiftUtil;
  private double scoreHubStartShotTimestamp = Double.NEGATIVE_INFINITY;

  private static final LoggedTunableNumber shootingDoneDelaySecs =
      new LoggedTunableNumber("Choreographer/ShootingDoneDelaySecs", 0.25);

  public Choreographer(
      Drive drive,
      Flywheel flywheel,
      Hood hood,
      Indexer indexer,
      Kicker kicker,
      Intake intake,
      BooleanSupplier driveAlignedForHubShot) {
    this.drive = drive;
    this.flywheel = flywheel;
    this.hood = hood;
    this.indexer = indexer;
    this.kicker = kicker;
    this.intake = intake;
    this.driveAlignedForHubShot = driveAlignedForHubShot;
    this.useHubShiftUtil = new LoggedNetworkBoolean("UseHubShiftUtil", true);
  }

  @Override
  public void periodic() {
    // Always log distance from hub regardless of enabled state
    Translation2d hubTarget =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    double distanceFromHub = drive.getPose().getTranslation().getDistance(hubTarget);
    Logger.recordOutput("Choreographer/DistanceFromHubMeters", distanceFromHub);

    cachedShotParams = ShotCalculator.getInstance().getParameters();
    cachedDriveAlignedForHubShot = driveAlignedForHubShot.getAsBoolean();

    if (!enabled || DriverStation.isDisabled()) {
      currentState = State.IDLE;
      logOutputs();
      return;
    }

    switch (currentGoal) {
      case IDLE -> handleIdle();
      case INTAKE -> handleIntake();
      case SCORE_HUB -> handleScoreHub();
      case CLIMB_EXTEND -> handleClimb(State.CLIMB_EXTENDING);
      case CLIMB_RETRACT -> handleClimb(State.CLIMB_RETRACTING);
    }

    logOutputs();
  }

  private void handleIdle() {
    flywheel.stop();
    hood.stow();
    indexer.stop();
    kicker.stop();
    currentState = State.IDLE;
  }

  private void handleIntake() {
    flywheel.stop();
    hood.stow();
    indexer.stop();
    kicker.stop();

    intake.deploy();
    intake.run();

    currentState = State.INTAKING;
  }

  private void handleScoreHub() {
    boolean hubActive =
        HubShiftUtil.getShiftedShiftInfo().active() || !useHubShiftUtil.getAsBoolean();

    cachedShotParams = ShotCalculator.getInstance().getParameters();

    flywheel.runVelocity(cachedShotParams.flywheelSpeedRPS());
    hood.setGoalAngleDeg(cachedShotParams.hoodAngleDeg());

    boolean flywheelReady = flywheel.atGoal();
    boolean hoodReady = hood.isAtGoal();
    boolean driveAlignedForHubShot = cachedDriveAlignedForHubShot;

    if (flywheelReady
        && hoodReady
        && driveAlignedForHubShot
        && cachedShotParams.isValid()
        && (hubActive || cachedShotParams.passing())) {
      indexer.run();
      kicker.run();
      currentState = State.SHOOTING;
    } else if (flywheelReady && hoodReady) {
      indexer.stop();
      kicker.stop();
      currentState = State.WAITING_FOR_ALIGNMENT;
    } else {
      indexer.stop();
      kicker.stop();
      currentState = State.SPINNING_UP;
    }
  }

  private void handleClimb(State climbState) {
    stopAll();
    currentState = climbState;
  }

  private void stopAll() {
    flywheel.stop();
    hood.stow();
    indexer.stop();
    kicker.stop();
  }

  private boolean hasShotParams() {
    return cachedShotParams != null && cachedShotParams.isValid();
  }

  public Rotation2d getTargetHeading() {
    return hasShotParams() ? cachedShotParams.driveAngle() : drive.getRotation();
  }

  private void setGoal(Goal goal) {
    if (goal == Goal.SCORE_HUB && currentGoal != Goal.SCORE_HUB) {
      scoreHubStartShotTimestamp = ShotCalculator.getInstance().getLastShotTimestampSeconds();
    }
    currentGoal = goal;
  }

  public Command setGoalCommand(Goal goal) {
    return runOnce(() -> setGoal(goal)).withName("Choreographer.setGoal(" + goal + ")");
  }

  /** Toggles whether the Choreographer controls subsystems. */
  public Command toggleEnabledCommand() {
    return runOnce(() -> this.enabled = !this.enabled).withName("Choreographer.toggleEnabled");
  }

  @AutoLogOutput(key = "Choreographer/ReadyToShoot")
  public boolean isReadyToShoot() {
    return currentState == State.SHOOTING;
  }

  @SuppressWarnings("unused")
  @AutoLogOutput(key = "Choreographer/DoneShooting")
  public boolean isDoneShooting() {
    if (true) return false;

    if (currentGoal != Goal.SCORE_HUB) {
      return false;
    }

    return isDoneShooting(
        scoreHubStartShotTimestamp,
        ShotCalculator.getInstance().getLastShotTimestampSeconds(),
        ShotCalculator.getInstance().getTimeSinceLastShotSeconds(),
        shootingDoneDelaySecs.get());
  }

  static boolean isDoneShooting(
      double scoreHubStartShotTimestamp,
      double lastShotTimestamp,
      double timeSinceLastShotSeconds,
      double shootingDoneDelaySeconds) {
    return lastShotTimestamp > scoreHubStartShotTimestamp
        && timeSinceLastShotSeconds >= shootingDoneDelaySeconds;
  }

  public void setCoastOverride(BooleanSupplier shouldCoast) {
    hood.setCoastOverride(shouldCoast);
    indexer.setCoastOverride(shouldCoast);
    kicker.setCoastOverride(shouldCoast);
    intake.setPivotCoastOverride(shouldCoast);
    intake.setRollersCoastOverride(shouldCoast);
  }

  private void logOutputs() {
    Logger.recordOutput("Choreographer/FlywheelReady", flywheel.atGoal());
    Logger.recordOutput("Choreographer/HoodReady", hood.isAtGoal());
    Logger.recordOutput("Choreographer/DriveAlignedForHubShot", cachedDriveAlignedForHubShot);
    Logger.recordOutput("Choreographer/IndexerRunning", indexer.isRunning());
    Logger.recordOutput("Choreographer/IndexerStalled", indexer.isStalled());
    Logger.recordOutput("Choreographer/KickerRunning", kicker.isRunning());
    Logger.recordOutput("Choreographer/IntakeRunning", intake.isRunning());
    Logger.recordOutput("Choreographer/HasShotParams", hasShotParams());
    Logger.recordOutput("Choreographer/Passing", hasShotParams() && cachedShotParams.passing());
    SmartDashboard.putBoolean(
        "Choreographer/Passing", hasShotParams() && cachedShotParams.passing());
    var shiftInfo = HubShiftUtil.getShiftedShiftInfo();
    Logger.recordOutput("Choreographer/HubActive", shiftInfo.active());
    SmartDashboard.putBoolean("Choreographer/HubActive", shiftInfo.active());
    Logger.recordOutput("Choreographer/HubShift", shiftInfo.currentShift().name());
    SmartDashboard.putString("Choreographer/HubShiftName", shiftInfo.currentShift().name());
    Logger.recordOutput("Choreographer/HubTimeRemaining", shiftInfo.remainingTime());
    SmartDashboard.putNumber("Choreographer/HubTimeRemaining", shiftInfo.remainingTime());
  }
}
