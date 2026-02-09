// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class ShotCalculator {
  private static ShotCalculator instance;

  private Rotation2d lastGoalHeading;
  private double lastHoodAngle;
  private Rotation2d goalHeading;
  private double hoodAngle = Double.NaN;

  private static final Transform2d robotToShooter =
      new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180));

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      boolean isValid, Rotation2d goalHeading, double hoodAngle, double flywheelSpeed) {}

  // Cache parameters
  private ShootingParameters latestParameters = null;

  /** When true, getParameters() returns fixed default values instead of computing from pose. */
  @Getter @Setter private boolean useDefaults = false;

  private static final LoggedTunableNumber defaultFlywheelSpeed =
      new LoggedTunableNumber("ShotCalculator/DefaultFlywheelSpeed", 50.0);
  private static final LoggedTunableNumber defaultHoodAngleDeg =
      new LoggedTunableNumber("ShotCalculator/DefaultHoodAngleDeg", 15.0);

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.34;
    maxDistance = 5.6;
    phaseDelay = 0.03;

    shotHoodAngleMap.put(1.0, Rotation2d.fromDegrees(0));
    shotHoodAngleMap.put(4.0, Rotation2d.fromDegrees(32));

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

    if (useDefaults) {
      Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
      latestParameters =
          new ShootingParameters(
              true,
              estimatedPose.getRotation(),
              Math.toRadians(defaultHoodAngleDeg.get()),
              defaultFlywheelSpeed.get());
      Logger.recordOutput("ShotCalculator/UsingDefaults", true);
      return latestParameters;
    }

    Logger.recordOutput("ShotCalculator/UsingDefaults", false);

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds fieldRelativeVelocity = RobotState.getInstance().getFieldVelocity();
    ChassisSpeeds robotRelativeVelocity =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeVelocity, estimatedPose.getRotation());
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate distance from robot to target
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d shooterPosition = estimatedPose.transformBy(robotToShooter);
    double shooterToTargetDistance = target.getDistance(shooterPosition.getTranslation());

    // Calculate field relative shooter velocity
    double shooterVelocityX = fieldRelativeVelocity.vxMetersPerSecond;
    double shooterVelocityY = fieldRelativeVelocity.vyMetersPerSecond;

    // Account for imparted velocity by robot to offset
    double timeOfFlight;
    Pose2d lookaheadPose = shooterPosition;
    double lookaheadShooterToTargetDistance = shooterToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadShooterToTargetDistance);
      double offsetX = shooterVelocityX * timeOfFlight;
      double offsetY = shooterVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              shooterPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              shooterPosition.getRotation());
      lookaheadShooterToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    Rotation2d angleToTarget = target.minus(lookaheadPose.getTranslation()).getAngle();
    goalHeading = angleToTarget.minus(robotToShooter.getRotation());

    hoodAngle = shotHoodAngleMap.get(lookaheadShooterToTargetDistance).getRadians();
    if (lastGoalHeading == null) lastGoalHeading = goalHeading;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

    lastGoalHeading = goalHeading;
    lastHoodAngle = hoodAngle;

    latestParameters =
        new ShootingParameters(
            lookaheadShooterToTargetDistance >= minDistance
                && lookaheadShooterToTargetDistance <= maxDistance,
            goalHeading,
            hoodAngle,
            shotFlywheelSpeedMap.get(lookaheadShooterToTargetDistance));

    // Log calculated values
    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/ShooterToTargetDistance", lookaheadShooterToTargetDistance);

    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}
