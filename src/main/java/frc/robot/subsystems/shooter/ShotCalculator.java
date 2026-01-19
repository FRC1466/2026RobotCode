// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {
  private static ShotCalculator instance;
  private static final Transform2d robotToShooter =
      new Transform2d(new Translation2d(0.15, 0.0), Rotation2d.fromDegrees(180));

  private final LinearFilter goalHeadingFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  private Rotation2d lastGoalHeading;
  private double lastHoodAngle;
  private Rotation2d goalHeading;
  private double hoodAngle = Double.NaN;
  private double goalHeadingVelocity;
  private double hoodVelocity;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      Rotation2d goalHeading,
      double goalHeadingVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed) {}

  // Cache parameters
  private ShootingParameters latestParameters = null;

  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    shotHoodAngleMap.put((double) 1, Rotation2d.fromDegrees(19));

    shotFlywheelSpeedMap.put(1.86, 45.0);
    shotFlywheelSpeedMap.put(1.75, 47.0);
    shotFlywheelSpeedMap.put(2.54, 51.0);
    shotFlywheelSpeedMap.put(3.22, 57.0);
    shotFlywheelSpeedMap.put(3.75, 61.0);

    timeOfFlightMap.put(1.86, 0.81);
    timeOfFlightMap.put(1.52, 0.86);
    timeOfFlightMap.put(2.54, .96);
    timeOfFlightMap.put(3.22, 1.3);
    timeOfFlightMap.put(3.75, 1.36);
  }

  public ShootingParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate distance from robot center to target for time-of-flight lookup
    Translation2d target = AllianceFlipUtil.apply(FieldConstants.hubCenter);
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    double robotToTargetDistance = target.getDistance(robotPose.getTranslation());

    // Get robot velocity and calculate lookahead for velocity compensation
    ChassisSpeeds robotVelocity = RobotState.getInstance().getFieldVelocity();
    double timeOfFlight = timeOfFlightMap.get(robotToTargetDistance);
    Translation2d robotLookaheadOffset =
        new Translation2d(
            robotVelocity.vxMetersPerSecond * timeOfFlight,
            robotVelocity.vyMetersPerSecond * timeOfFlight);

    // Calculate shooter position after robot has moved
    Pose2d lookaheadRobotPose =
        new Pose2d(robotPose.getTranslation().plus(robotLookaheadOffset), robotPose.getRotation());
    Pose2d shooterPosition = lookaheadRobotPose.transformBy(robotToShooter);
    double shooterToTargetDistance = target.getDistance(shooterPosition.getTranslation());

    // Calculate parameters for the shot
    goalHeading =
        target
            .minus(shooterPosition.getTranslation())
            .getAngle()
            .minus(robotToShooter.getRotation());
    hoodAngle = shotHoodAngleMap.get(shooterToTargetDistance).getRadians();
    if (lastGoalHeading == null) lastGoalHeading = goalHeading;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    goalHeadingVelocity =
        goalHeadingFilter.calculate(
            goalHeading.minus(lastGoalHeading).getRadians() / Constants.loopPeriodSecs);
    hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastGoalHeading = goalHeading;
    lastHoodAngle = hoodAngle;
    latestParameters =
        new ShootingParameters(
            goalHeading,
            goalHeadingVelocity,
            hoodAngle,
            hoodVelocity,
            shotFlywheelSpeedMap.get(shooterToTargetDistance));

    // Log calculated values
    Logger.recordOutput("ShotCalculator/LookaheadPose", shooterPosition);
    Logger.recordOutput("ShotCalculator/ShooterToTargetDistance", shooterToTargetDistance);

    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}
