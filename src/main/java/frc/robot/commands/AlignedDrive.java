// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import frc.robot.util.ShooterModel;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Command factory for auto-aligning the robot to various targets while driving. Supports dynamic
 * target selection and time-of-flight prediction.
 */
public class AlignedDrive {

  /** Target position for the primary goal */
  private static final double primaryTargetX = 4.5;

  private static final double primaryTargetY = Constants.fieldWidthMeters / 2;

  /** X threshold to switch from primary to alternate targets */
  private static final double alternateTargetThresholdX = 3.75;

  /** Alternate target positions (left and right) */
  private static final double alternateTargetX = 2.0;

  private static final double alternateTargetYOffset = 1; // Offset from center

  /** Number of iterations for time-of-flight convergence */
  private static final int tofIterations = 3;

  private AlignedDrive() {}

  /**
   * Creates a command that drives with joystick input while auto-aligning to the appropriate
   * target.
   *
   * @param drive The drive subsystem
   * @param xSupplier Supplier for forward/backward joystick input
   * @param ySupplier Supplier for left/right joystick input
   * @param speedMultiplier Speed multiplier for driving (e.g., 0.75 for 75% speed)
   * @return Command that drives while auto-aligning
   */
  public static Command autoAlign(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, double speedMultiplier) {
    return DriveCommands.joystickDriveAtAngle(
        drive,
        () -> -speedMultiplier * xSupplier.getAsDouble(),
        () -> -speedMultiplier * ySupplier.getAsDouble(),
        () -> calculateTargetAngle(drive));
  }

  /**
   * Creates a command that drives with joystick input while auto-aligning to the appropriate
   * target. Uses default 75% speed multiplier.
   *
   * @param drive The drive subsystem
   * @param xSupplier Supplier for forward/backward joystick input
   * @param ySupplier Supplier for left/right joystick input
   * @return Command that drives while auto-aligning
   */
  public static Command autoAlign(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return autoAlign(drive, xSupplier, ySupplier, 0.75);
  }

  /**
   * Calculates the target angle to aim at, accounting for time-of-flight prediction. Automatically
   * selects the appropriate target based on robot position.
   *
   * @param drive The drive subsystem
   * @return The target angle to face
   */
  private static Rotation2d calculateTargetAngle(Drive drive) {
    Pose2d currentPose = drive.getPose();

    // Determine which target to aim at
    Pose2d targetPosition = selectTarget(currentPose);
    double targetX = targetPosition.getX();
    double targetY = targetPosition.getY();

    // Iteratively converge on the correct angle with time-of-flight prediction
    double timeOfFlight = 0;
    Pose2d futurePose = currentPose;

    for (int i = 0; i < tofIterations; i++) {
      futurePose = drive.getFuturePose(timeOfFlight);
      double distance = Math.hypot(targetX - futurePose.getX(), targetY - futurePose.getY());
      timeOfFlight = ShooterModel.getTimeOfFlight(distance);
    }

    // Calculate angle to target from converged future position
    Rotation2d targetAngle =
        Rotation2d.fromRadians(
            Math.atan2(targetY - futurePose.getY(), targetX - futurePose.getX()));

    // Log debug information
    Logger.recordOutput("AutoAlign/TargetPose", GeomUtil.withCoords(targetX, targetY));
    Logger.recordOutput("AutoAlign/TimeOfFlight", timeOfFlight);
    Logger.recordOutput("AutoAlign/FuturePose", futurePose);
    Logger.recordOutput("AutoAlign/TargetAngle", targetAngle);

    return targetAngle;
  }

  /**
   * Selects the appropriate target based on robot position.
   *
   * @param currentPose Current robot pose
   * @return Target position as a Pose2d (only x and y are used)
   */
  private static Pose2d selectTarget(Pose2d currentPose) {
    double robotX = currentPose.getX();

    // If robot is past the threshold, use alternate targets
    if (robotX > alternateTargetThresholdX) {
      double centerY = Constants.fieldWidthMeters / 2;
      double leftTargetY = centerY + alternateTargetYOffset;
      double rightTargetY = centerY - alternateTargetYOffset;

      // Calculate distances to both alternate targets
      double distanceToLeft =
          Math.hypot(alternateTargetX - currentPose.getX(), leftTargetY - currentPose.getY());
      double distanceToRight =
          Math.hypot(alternateTargetX - currentPose.getX(), rightTargetY - currentPose.getY());

      // Choose the closer target
      if (distanceToLeft < distanceToRight) {
        Logger.recordOutput("AutoAlign/SelectedTarget", "AlternateLeft");
        return new Pose2d(alternateTargetX, leftTargetY, new Rotation2d());
      } else {
        Logger.recordOutput("AutoAlign/SelectedTarget", "AlternateRight");
        return new Pose2d(alternateTargetX, rightTargetY, new Rotation2d());
      }
    } else {
      // Use primary target
      Logger.recordOutput("AutoAlign/SelectedTarget", "Primary");
      return new Pose2d(primaryTargetX, primaryTargetY, new Rotation2d());
    }
  }
}
