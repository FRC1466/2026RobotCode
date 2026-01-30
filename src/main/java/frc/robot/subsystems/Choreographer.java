// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Choreographer (Superstructure) - Coordinates subsystems via a high-level state machine. Exposes
 * target heading for drive integration via {@link #getTargetHeading()}.
 */
public class Choreographer extends SubsystemBase {

  /** High-level operator intention. */
  public enum Goal {
    IDLE,
    INTAKE,
    SCORE_HUB,
    CLIMB_EXTEND,
    CLIMB_RETRACT
  }

  /** Actual robot state based on sensor feedback. */
  public enum State {
    IDLE,
    INTAKING,
    SPINNING_UP,
    READY_TO_SHOOT,
    CLIMB_EXTENDING,
    CLIMB_EXTENDED,
    CLIMB_RETRACTING,
    CLIMBED
  }

  private static final double DRIVE_ALIGNED_TOLERANCE_RAD = Units.degreesToRadians(3.0);

  @Getter
  @AutoLogOutput(key = "Choreographer/Goal")
  private Goal currentGoal = Goal.IDLE;

  @Getter
  @AutoLogOutput(key = "Choreographer/State")
  private State currentState = State.IDLE;

  private final Drive drive;
  private final Flywheel flywheel;
  private final Hood hood;
  private ShootingParameters cachedShotParams = null;

  public Choreographer(Drive drive, Flywheel flywheel, Hood hood) {
    this.drive = drive;
    this.flywheel = flywheel;
    this.hood = hood;
  }

  @Override
  public void periodic() {
    cachedShotParams = ShotCalculator.getInstance().getParameters();

    switch (currentGoal) {
      case IDLE, INTAKE -> {
        currentState = currentGoal == Goal.INTAKE ? State.INTAKING : State.IDLE;
        flywheel.stop();
        hood.stow();
      }
      case SCORE_HUB -> {
        if (cachedShotParams != null && cachedShotParams.isValid()) {
          flywheel.runVelocity(cachedShotParams.flywheelSpeed());
          hood.setGoalAngleDeg(Units.radiansToDegrees(cachedShotParams.hoodAngle()));
          currentState = isAllReady() ? State.READY_TO_SHOOT : State.SPINNING_UP;
        } else {
          flywheel.stop();
          hood.stow();
          currentState = State.SPINNING_UP;
        }
      }
      case CLIMB_EXTEND -> {
        flywheel.stop();
        hood.stow();
        currentState = State.CLIMB_EXTENDING;
      }
      case CLIMB_RETRACT -> {
        flywheel.stop();
        hood.stow();
        currentState = State.CLIMB_RETRACTING;
      }
    }

    Logger.recordOutput("Choreographer/FlywheelReady", flywheel.isAtGoal());
    Logger.recordOutput("Choreographer/HoodReady", hood.isAtGoal());
    Logger.recordOutput("Choreographer/DriveAligned", isDriveAligned());
  }

  private boolean isAllReady() {
    return flywheel.isAtGoal() && hood.isAtGoal() && isDriveAligned();
  }

  private boolean isDriveAligned() {
    if (cachedShotParams == null || !cachedShotParams.isValid()) return false;
    double headingError =
        Math.abs(drive.getRotation().minus(cachedShotParams.goalHeading()).getRadians());
    return headingError < DRIVE_ALIGNED_TOLERANCE_RAD
        && Math.abs(drive.getChassisSpeeds().omegaRadiansPerSecond) < 0.5;
  }

  /** Returns target heading for drive to track, or current heading if no valid shot. */
  public Rotation2d getTargetHeading() {
    return (cachedShotParams != null && cachedShotParams.isValid())
        ? cachedShotParams.goalHeading()
        : drive.getRotation();
  }

  public Command setGoalCommand(Goal goal) {
    return runOnce(() -> this.currentGoal = goal).withName("Choreographer.setGoal(" + goal + ")");
  }

  @AutoLogOutput(key = "Choreographer/ReadyToShoot")
  public boolean isReadyToShoot() {
    return currentState == State.READY_TO_SHOOT;
  }

  public void setCoastOverride(BooleanSupplier shouldCoast) {
    flywheel.setCoastOverride(shouldCoast);
    hood.setCoastOverride(shouldCoast);
  }
}
