// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class ShotCalculator {
  private static ShotCalculator instance;

  private static final Transform3d robotToShooter =
      new Transform3d(0, 0, 0, new Rotation3d(0.0, 0.0, 0.0));

  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
  private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.8 / Constants.loopPeriodSecs));

  private double lastHoodAngle;
  private Rotation2d lastDriveAngle;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing) {}

  // Cache parameters
  private ShootingParameters latestParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static double passingMinDistance;
  private static double passingMaxDistance;

  // Shooting maps
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Passing maps
  private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Passing target
  private static final double hubPassLine =
      FieldConstants.LinesHorizontal.rightBumpStart - Drive.DRIVE_BASE_RADIUS;
  private static final double xPassTarget = Units.inchesToMeters(25);
  private static final double yPassTarget = Units.inchesToMeters(50);

  // Bad-box bounds (min x, max x, min y, max y) — field-relative, blue-origin
  private static final double[] towerBound = {
    0, Units.inchesToMeters(129), Units.inchesToMeters(46), Units.inchesToMeters(168)
  };
  private static final double[] nearHubBound = {
    FieldConstants.LinesVertical.neutralZoneNear,
    FieldConstants.LinesVertical.neutralZoneNear + Units.inchesToMeters(65),
    FieldConstants.LinesHorizontal.rightBumpStart,
    FieldConstants.LinesHorizontal.leftBumpEnd
  };
  private static final double[] farHubBound = {
    FieldConstants.LinesVertical.oppAllianceZone,
    FieldConstants.fieldLength,
    FieldConstants.LinesHorizontal.rightBumpStart,
    FieldConstants.LinesHorizontal.leftBumpEnd
  };

  static {
    minDistance = 1.34;
    maxDistance = 5.60;
    phaseDelay = 0.03;
    passingMinDistance = 0.0;
    passingMaxDistance = 100000;

    hoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    hoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    hoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    hoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    hoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    hoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    hoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    hoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    flywheelSpeedMap.put(1.34, 210.0);
    flywheelSpeedMap.put(1.78, 220.0);
    flywheelSpeedMap.put(2.17, 220.0);
    flywheelSpeedMap.put(2.81, 230.0);
    flywheelSpeedMap.put(3.82, 250.0);
    flywheelSpeedMap.put(4.09, 255.0);
    flywheelSpeedMap.put(4.40, 260.0);
    flywheelSpeedMap.put(4.77, 265.0);
    flywheelSpeedMap.put(5.57, 275.0);
    flywheelSpeedMap.put(5.60, 290.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);

    // TODO: tune passing maps
    passingHoodAngleMap.put(passingMinDistance, Rotation2d.fromDegrees(0.0));
    passingHoodAngleMap.put(passingMaxDistance, Rotation2d.fromDegrees(0.0));
    passingFlywheelSpeedMap.put(passingMinDistance, 0.0);
    passingFlywheelSpeedMap.put(passingMaxDistance, 0.0);
    passingTimeOfFlightMap.put(passingMinDistance, 0.0);
    passingTimeOfFlightMap.put(passingMaxDistance, 0.0);
  }

  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDistance);
  }

  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDistance);
  }

  public ShootingParameters getParameters() {
    boolean passing =
        AllianceFlipUtil.applyX(RobotState.getInstance().getEstimatedPose().getX())
            > FieldConstants.LinesVertical.hubCenter;
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getFieldVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate target
    Translation2d target =
        AllianceFlipUtil.apply(
            passing ? getPassingTarget() : FieldConstants.Hub.topCenterPoint.toTranslation2d());

    // Calculate distance from shooter to target
    Pose2d shooterPosition = estimatedPose.transformBy(robotToShooter.toTransform2d());
    double shooterToTargetDistance = target.getDistance(shooterPosition.getTranslation());

    // Calculate field relative shooter velocity
    double shooterVelocityX = RobotState.getInstance().getFieldVelocity().vxMetersPerSecond;
    double shooterVelocityY = RobotState.getInstance().getFieldVelocity().vyMetersPerSecond;

    // Account for imparted velocity by robot (shooter) to offset
    double timeOfFlight =
        passing
            ? passingTimeOfFlightMap.get(shooterToTargetDistance)
            : timeOfFlightMap.get(shooterToTargetDistance);
    Pose2d lookaheadPose = shooterPosition;
    double lookaheadShooterToTargetDistance = shooterToTargetDistance;

    for (int i = 0; i < 20; i++) {
      timeOfFlight =
          passing
              ? passingTimeOfFlightMap.get(lookaheadShooterToTargetDistance)
              : timeOfFlightMap.get(lookaheadShooterToTargetDistance);
      double offsetX = shooterVelocityX * timeOfFlight;
      double offsetY = shooterVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              shooterPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              shooterPosition.getRotation());
      lookaheadShooterToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Account for shooter being off center
    Pose2d lookaheadRobotPose = lookaheadPose.transformBy(robotToShooter.toTransform2d().inverse());
    Rotation2d driveAngle = getDriveAngleWithShooterOffset(lookaheadRobotPose, target);

    // Calculate remaining parameters
    double hoodAngle =
        passing
            ? passingHoodAngleMap.get(lookaheadShooterToTargetDistance).getRadians()
            : hoodAngleMap.get(lookaheadShooterToTargetDistance).getRadians();
    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    double hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastHoodAngle = hoodAngle;
    double driveVelocity =
        driveAngleFilter.calculate(
            driveAngle.minus(lastDriveAngle).getRadians() / Constants.loopPeriodSecs);
    lastDriveAngle = driveAngle;

    // Check if inside a bad zone
    var flippedPose = AllianceFlipUtil.apply(estimatedPose);
    Translation2d fp = flippedPose.getTranslation();
    boolean insideTowerBadBox = inBounds(fp, towerBound);
    boolean behindNearHub = inBounds(fp, nearHubBound);
    boolean behindFarHub = inBounds(fp, farHubBound);
    boolean outsideOfBadBoxes = !(insideTowerBadBox || behindNearHub || behindFarHub);

    latestParameters =
        new ShootingParameters(
            outsideOfBadBoxes
                && lookaheadShooterToTargetDistance >= (passing ? passingMinDistance : minDistance)
                && lookaheadShooterToTargetDistance <= (passing ? passingMaxDistance : maxDistance),
            driveAngle,
            driveVelocity,
            hoodAngle,
            hoodVelocity,
            passing
                ? passingFlywheelSpeedMap.get(lookaheadShooterToTargetDistance)
                : flywheelSpeedMap.get(lookaheadShooterToTargetDistance),
            lookaheadShooterToTargetDistance,
            shooterToTargetDistance,
            timeOfFlight,
            passing);

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput(
        "LaunchCalculator/ShooterToTargetDistance", lookaheadShooterToTargetDistance);

    return latestParameters;
  }

  private static boolean inBounds(Translation2d point, double[] bound) {
    return point.getX() >= bound[0]
        && point.getX() <= bound[1]
        && point.getY() >= bound[2]
        && point.getY() <= bound[3];
  }

  public Translation2d getPassingTarget() {
    double flippedY = AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose()).getY();
    boolean mirror = flippedY > FieldConstants.LinesHorizontal.center;

    if (FieldConstants.fieldWidth - hubPassLine > flippedY && flippedY > hubPassLine) {
      double interpolateZoneAmount =
          ((mirror ? FieldConstants.fieldWidth - flippedY : flippedY) - hubPassLine)
              / (FieldConstants.LinesHorizontal.center - hubPassLine);
      double unflippedPoseY =
          mirror
              ? FieldConstants.fieldWidth
                  - MathUtil.interpolate(yPassTarget, passingMinDistance, interpolateZoneAmount)
              : MathUtil.interpolate(yPassTarget, passingMinDistance, interpolateZoneAmount);
      return AllianceFlipUtil.apply(new Translation2d(xPassTarget, unflippedPoseY));
    }

    return AllianceFlipUtil.apply(
        new Translation2d(
            xPassTarget, mirror ? FieldConstants.fieldWidth - yPassTarget : yPassTarget));
  }

  private static Rotation2d getDriveAngleWithShooterOffset(Pose2d robotPose, Translation2d target) {
    Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
    Rotation2d hubAngle =
        new Rotation2d(
            Math.asin(
                MathUtil.clamp(
                    robotToShooter.getTranslation().getY()
                        / target.getDistance(robotPose.getTranslation()),
                    -1.0,
                    1.0)));
    Rotation2d driveAngle =
        fieldToHubAngle.plus(hubAngle).plus(robotToShooter.getRotation().toRotation2d());
    return driveAngle;
  }

  public double getNaiveTOF(double distance) {
    return timeOfFlightMap.get(distance);
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }

  /**
   * Returns the Pose2d that correctly aims the robot at the goal for a given robot translation.
   *
   * @param robotTranslation The translation of the center of the robot.
   * @return The target pose for the aimed robot.
   */
  public static Pose2d getStationaryAimedPose(Translation2d robotTranslation) {
    // Calculate target
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    return new Pose2d(
        robotTranslation, getDriveAngleWithShooterOffset(robotTranslation.toPose2d(), target));
  }
}
