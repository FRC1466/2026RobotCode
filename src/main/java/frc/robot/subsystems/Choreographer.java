// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Choreographer extends SubsystemBase {

  /** High-level operator intention. */
  public enum Goal {
    IDLE,
    INTAKE,
    SCORE_HUB,
    CLIMB_EXTEND,
    CLIMB_RETRACT
  }

  /** Observed robot state derived from sensor feedback. */
  public enum State {
    IDLE,
    INTAKING,
    SPINNING_UP,
    READY_TO_SHOOT,
    SHOOTING,
    CLIMB_EXTENDING,
    CLIMB_RETRACTING
  }

  private static final LoggedTunableNumber driveAlignedToleranceDeg =
      new LoggedTunableNumber("Choreographer/DriveAlignedToleranceDeg", 3.0);
  private static final LoggedTunableNumber driveAlignedOmegaTolerance =
      new LoggedTunableNumber("Choreographer/DriveAlignedOmegaTolerance", 0.5);

  @Getter
  @AutoLogOutput(key = "Choreographer/Goal")
  private Goal currentGoal = Goal.IDLE;

  @Getter
  @AutoLogOutput(key = "Choreographer/State")
  private State currentState = State.IDLE;

  private final Drive drive;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Spindexer spindexer;
  private final Kicker kicker;

  private ShootingParameters cachedShotParams = null;

  public Choreographer(
      Drive drive, Flywheel flywheel, Hood hood, Spindexer spindexer, Kicker kicker) {
    this.drive = drive;
    this.flywheel = flywheel;
    this.hood = hood;
    this.spindexer = spindexer;
    this.kicker = kicker;
  }

  @Override
  public void periodic() {
    cachedShotParams = ShotCalculator.getInstance().getParameters();

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
    stopAll();
    currentState = State.IDLE;
  }

  private void handleIntake() {
    flywheel.stop();
    hood.stow();
    spindexer.stop();
    kicker.stop();
    currentState = State.INTAKING;
  }

  private void handleScoreHub() {
    if (cachedShotParams == null || !cachedShotParams.isValid()) {
      stopAll();
      currentState = State.SPINNING_UP;
      return;
    }

    flywheel.runVelocity(cachedShotParams.flywheelSpeed());
    hood.setGoalAngleDeg(Units.radiansToDegrees(cachedShotParams.hoodAngle()));

    boolean flywheelReady = flywheel.isAtGoal();
    boolean hoodReady = hood.isAtGoal();
    boolean driveAligned = isDriveAligned();

    if (flywheelReady && hoodReady && driveAligned) {
      spindexer.run();
      kicker.run();
      currentState = State.SHOOTING;
    } else if (flywheelReady && hoodReady) {
      spindexer.stop();
      kicker.stop();
      currentState = State.READY_TO_SHOOT;
    } else {
      spindexer.stop();
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
    spindexer.stop();
    kicker.stop();
  }

  private boolean isDriveAligned() {
    if (cachedShotParams == null || !cachedShotParams.isValid()) return false;
    double headingErrorRad =
        Math.abs(drive.getRotation().minus(cachedShotParams.goalHeading()).getRadians());
    return headingErrorRad < Units.degreesToRadians(driveAlignedToleranceDeg.get())
        && Math.abs(drive.getChassisSpeeds().omegaRadiansPerSecond)
            < driveAlignedOmegaTolerance.get();
  }

  private boolean hasShotParams() {
    return cachedShotParams != null && cachedShotParams.isValid();
  }

  /** Returns target heading for drive to track, or current heading if no valid shot. */
  public Rotation2d getTargetHeading() {
    return hasShotParams() ? cachedShotParams.goalHeading() : drive.getRotation();
  }

  public Command setGoalCommand(Goal goal) {
    return runOnce(() -> this.currentGoal = goal).withName("Choreographer.setGoal(" + goal + ")");
  }

  @AutoLogOutput(key = "Choreographer/ReadyToShoot")
  public boolean isReadyToShoot() {
    return currentState == State.READY_TO_SHOOT || currentState == State.SHOOTING;
  }

  public void setCoastOverride(BooleanSupplier shouldCoast) {
    flywheel.setCoastOverride(shouldCoast);
    hood.setCoastOverride(shouldCoast);
    spindexer.setCoastOverride(shouldCoast);
    kicker.setCoastOverride(shouldCoast);
  }

  private void logOutputs() {
    Logger.recordOutput("Choreographer/FlywheelReady", flywheel.isAtGoal());
    Logger.recordOutput("Choreographer/HoodReady", hood.isAtGoal());
    Logger.recordOutput("Choreographer/DriveAligned", isDriveAligned());
    Logger.recordOutput("Choreographer/SpindexerRunning", spindexer.isRunning());
    Logger.recordOutput("Choreographer/SpindexerStalled", spindexer.isStalled());
    Logger.recordOutput("Choreographer/KickerRunning", kicker.isRunning());
    Logger.recordOutput("Choreographer/HasShotParams", hasShotParams());
  }
}
