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
    shotHoodAngleMap.put(1.45, Rotation2d.fromDegrees(19.0));
    shotHoodAngleMap.put(1.75, Rotation2d.fromDegrees(21.0));
    shotHoodAngleMap.put(2.15, Rotation2d.fromDegrees(22.0));
    shotHoodAngleMap.put(2.50, Rotation2d.fromDegrees(23.0));
    shotHoodAngleMap.put(2.84, Rotation2d.fromDegrees(24.0));
    shotHoodAngleMap.put(3.15, Rotation2d.fromDegrees(25.5));
    shotHoodAngleMap.put(3.58, Rotation2d.fromDegrees(26.5));
    shotHoodAngleMap.put(4.16, Rotation2d.fromDegrees(29.0));
    shotHoodAngleMap.put(4.43, Rotation2d.fromDegrees(30.5));
    shotHoodAngleMap.put(5.28, Rotation2d.fromDegrees(34.0));

    shotFlywheelSpeedMap.put(1.45, 45.0);
    shotFlywheelSpeedMap.put(1.75, 50.0);
    shotFlywheelSpeedMap.put(2.15, 55.0);
    shotFlywheelSpeedMap.put(2.50, 58.0);
    shotFlywheelSpeedMap.put(2.84, 59.0);
    shotFlywheelSpeedMap.put(3.15, 65.0);
    shotFlywheelSpeedMap.put(3.58, 75.0);
    shotFlywheelSpeedMap.put(4.16, 88.0);
    shotFlywheelSpeedMap.put(4.43, 95.0);
    shotFlywheelSpeedMap.put(5.28, 105.0);

    timeOfFlightMap.put(1.64227, 0.93);
    timeOfFlightMap.put(2.859544, 1.0);
    timeOfFlightMap.put(4.27071, 1.05);
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
