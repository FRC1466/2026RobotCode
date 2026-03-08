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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class ShotCalculator {
  private static ShotCalculator instance;

  private double flywheelSpeedOffsetRPS = 0.0;
  private double flywheelSpeedOffsetPercent = 0.0;

  private static final Transform3d robotToShooter =
      new Transform3d(0, 0, 0, new Rotation3d(0.0, 0.0, 0.0));

  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
  private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.8 / Constants.loopPeriodSecs));

  private double lastHoodAngle = Double.NaN;
  private Rotation2d lastDriveAngle;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  @AutoLogOutput(key = "ShotCalculator/FlywheelSpeedOffsetRPS")
  public double getFlywheelSpeedOffsetRPS() {
    return flywheelSpeedOffsetRPS;
  }

  @AutoLogOutput(key = "ShotCalculator/FlywheelSpeedOffsetPercent")
  public double getFlywheelSpeedOffsetPercent() {
    return flywheelSpeedOffsetPercent;
  }

  public record ShootingParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocity,
      double hoodAngleDeg,
      double hoodVelocityDegPerSec,
      double flywheelSpeedRPS,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing) {}

  // Cache parameters
  private ShootingParameters latestParameters = null;

  /**
   * When true, getParameters() ignores vision/pose and returns hub-preset values at
   * hubPresetDistance with the robot's current heading — no auto-rotation. Toggle via right bumper
   * on controller or the SmartDashboard key "ShotCalculator/HubPresetOverride".
   */
  private boolean hubPresetOverride = false;

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

  // Presets
  public static final double hubPresetDistance = 0.96;
  public static final double towerPresetDistance = 2.5;
  public static final double trenchPresetDistance = 3.03;
  public static final double outpostPresetDistance = 4.84;
  public static final LaunchPreset hubPreset;
  public static final LaunchPreset towerPreset;
  public static final LaunchPreset trenchPreset;
  public static final LaunchPreset outpostPreset;
  public static final LaunchPreset hoodMinPreset =
      new LaunchPreset(
          new LoggedTunableNumber("ShotCalculator/Presets/HoodMin/HoodAngle", Hood.minAngleDeg),
          new LoggedTunableNumber("ShotCalculator/Presets/HoodMin/FlywheelSpeed", 100));
  public static final LaunchPreset hoodMaxPreset =
      new LaunchPreset(
          new LoggedTunableNumber("ShotCalculator/Presets/HoodMax/HoodAngle", Hood.maxAngleDeg),
          new LoggedTunableNumber("ShotCalculator/Presets/HoodMax/FlywheelSpeed", 100));

  public static record LaunchPreset(
      LoggedTunableNumber hoodAngleDeg, LoggedTunableNumber flywheelSpeed) {}

  // Shooting map data — distances (m), default hood angles (deg), flywheel speeds (RPS), and
  // time-of-flight (s). Hood angles and flywheel speeds are exposed as LoggedTunableNumbers so
  // they can be edited live in tuning mode without redeploying.
  private static final double[] mapDistances = {1.969, 2.185, 2.452, 3.701, 4.06, 4.68};
  private static final double[] defaultMapHoodAngles = {
    Hood.minAngleDeg, Hood.minAngleDeg, Hood.minAngleDeg, 5.0, 8.0, 10.0
  };
  private static final double[] defaultMapFlywheelSpeeds = {31.0, 32.0, 33.0, 36.0, 37.75, 39.0};
  private static final double[] mapTimeOfFlights = {1.2, 1.2, 1.0, 1.61, 1.4, 1.1};
  private static final LoggedTunableNumber[] mapHoodAnglesDeg;
  private static final LoggedTunableNumber[] mapFlywheelSpeedsRPS;
  private static final int MAP_CONSUMER_ID = ShotCalculator.class.hashCode();

  static {
    minDistance = 1.34;
    maxDistance = 5.60;
    phaseDelay = 0.03;
    passingMinDistance = 0.0;
    passingMaxDistance = 100000;

    mapHoodAnglesDeg = new LoggedTunableNumber[mapDistances.length];
    mapFlywheelSpeedsRPS = new LoggedTunableNumber[mapDistances.length];
    for (int i = 0; i < mapDistances.length; i++) {
      mapHoodAnglesDeg[i] =
          new LoggedTunableNumber(
              String.format("ShotCalculator/Map/%.3fm/HoodAngleDeg", mapDistances[i]),
              defaultMapHoodAngles[i]);
      mapFlywheelSpeedsRPS[i] =
          new LoggedTunableNumber(
              String.format("ShotCalculator/Map/%.3fm/FlywheelSpeedRPS", mapDistances[i]),
              defaultMapFlywheelSpeeds[i]);
      timeOfFlightMap.put(mapDistances[i], mapTimeOfFlights[i]);
    }
    rebuildShootingMaps();

    // TODO: tune passing maps
    passingHoodAngleMap.put(passingMinDistance, Rotation2d.fromDegrees(0.0));
    passingHoodAngleMap.put(passingMaxDistance, Rotation2d.fromDegrees(0.0));
    passingFlywheelSpeedMap.put(passingMinDistance, 0.0);
    passingFlywheelSpeedMap.put(passingMaxDistance, 0.0);
    passingTimeOfFlightMap.put(passingMinDistance, 0.0);
    passingTimeOfFlightMap.put(passingMaxDistance, 0.0);

    hubPreset =
        new LaunchPreset(
            new LoggedTunableNumber("ShotCalculator/Presets/Hub/HoodAngle", 0),
            new LoggedTunableNumber("ShotCalculator/Presets/Hub/FlywheelSpeed", 33));
    towerPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "ShotCalculator/Presets/Tower/HoodAngle",
                hoodAngleMap.get(towerPresetDistance).getDegrees()),
            new LoggedTunableNumber(
                "ShotCalculator/Presets/Tower/FlywheelSpeed",
                flywheelSpeedMap.get(towerPresetDistance)));
    trenchPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "ShotCalculator/Presets/Trench/HoodAngle",
                hoodAngleMap.get(trenchPresetDistance).getDegrees()),
            new LoggedTunableNumber(
                "ShotCalculator/Presets/Trench/FlywheelSpeed",
                flywheelSpeedMap.get(trenchPresetDistance)));
    outpostPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "ShotCalculator/Presets/Outpost/HoodAngle",
                hoodAngleMap.get(outpostPresetDistance).getDegrees()),
            new LoggedTunableNumber(
                "ShotCalculator/Presets/Outpost/FlywheelSpeed",
                flywheelSpeedMap.get(outpostPresetDistance)));
  }

  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDistance);
  }

  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDistance);
  }

  /** Rebuilds the hood-angle and flywheel-speed interpolation maps from the current tunable values. */
  private static void rebuildShootingMaps() {
    hoodAngleMap.clear();
    flywheelSpeedMap.clear();
    for (int i = 0; i < mapDistances.length; i++) {
      hoodAngleMap.put(mapDistances[i], Rotation2d.fromDegrees(mapHoodAnglesDeg[i].get()));
      flywheelSpeedMap.put(mapDistances[i], mapFlywheelSpeedsRPS[i].get());
    }
  }

  /**
   * Returns a Pose2d positioned {@code distance} metres in front of the alliance hub, aimed
   * directly at it. Useful for driving to a known shot distance during tuning.
   *
   * @param distance shooter-to-hub distance in metres
   * @return aimed Pose2d at the requested distance
   */
  public static Pose2d getAimedPoseAtDistance(double distance) {
    Translation2d hubCenterBlue = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    Translation2d robotBlue =
        new Translation2d(hubCenterBlue.getX() - distance, hubCenterBlue.getY());
    return getStationaryAimedPose(AllianceFlipUtil.apply(robotBlue));
  }

  /** Toggles hub-preset override on/off. Returns the new state. */
  public boolean toggleHubPresetOverride() {
    hubPresetOverride = !hubPresetOverride;
    SmartDashboard.putBoolean("ShotCalculator/HubPresetOverride", hubPresetOverride);
    Logger.recordOutput("ShotCalculator/HubPresetOverride", hubPresetOverride);
    return hubPresetOverride;
  }

  public boolean isHubPresetOverride() {
    return hubPresetOverride;
  }

  public void dropHood() {
    if (latestParameters != null) {
      latestParameters =
          new ShootingParameters(
              latestParameters.isValid(),
              latestParameters.driveAngle(),
              latestParameters.driveVelocity(),
              Hood.minAngleDeg,
              latestParameters.hoodVelocityDegPerSec(),
              latestParameters.flywheelSpeedRPS(),
              latestParameters.distance(),
              latestParameters.distanceNoLookahead(),
              latestParameters.timeOfFlight(),
              latestParameters.passing());
    }
  }

  double applyFlywheelSpeedOffsets(double baseFlywheelSpeedRPS) {
    return (baseFlywheelSpeedRPS * (1.0 + flywheelSpeedOffsetPercent)) + flywheelSpeedOffsetRPS;
  }

  public ShootingParameters getParameters() {
    boolean passing =
        AllianceFlipUtil.applyX(RobotState.getInstance().getEstimatedPose().getX())
            > FieldConstants.LinesVertical.hubCenter;
    if (latestParameters != null) {
      return latestParameters;
    }

    // Rebuild interpolation maps if any tunable map values have changed.
    boolean mapChanged = false;
    for (LoggedTunableNumber n : mapHoodAnglesDeg) mapChanged |= n.hasChanged(MAP_CONSUMER_ID);
    for (LoggedTunableNumber n : mapFlywheelSpeedsRPS) mapChanged |= n.hasChanged(MAP_CONSUMER_ID);
    if (mapChanged) rebuildShootingMaps();

    // Hub-preset override: ignore vision/pose, use preset values at hubPresetDistance and keep
    // the robot's current heading so there is no auto-rotation.
    if (hubPresetOverride) {
      Rotation2d currentHeading = RobotState.getInstance().getEstimatedPose().getRotation();
      double d = hubPresetDistance;
      latestParameters =
          new ShootingParameters(
              true,
              currentHeading,
              0.0,
              hubPreset.hoodAngleDeg().get(),
              0.0,
              applyFlywheelSpeedOffsets(hubPreset.flywheelSpeed().get()),
              d,
              d,
              timeOfFlightMap.get(d),
              false);
      Logger.recordOutput("ShotCalculator/HubPresetOverride", true);
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
    var robotVelocity = RobotState.getInstance().getFieldVelocity();
    var robotAngle = RobotState.getInstance().getEstimatedPose().getRotation();
    ChassisSpeeds shooterVelocity =
        GeomUtil.transformVelocity(
            robotVelocity, robotToShooter.getTranslation().toTranslation2d(), robotAngle);

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
      double offsetX = shooterVelocity.vxMetersPerSecond * timeOfFlight;
      double offsetY = shooterVelocity.vyMetersPerSecond * timeOfFlight;
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
    double hoodAngleDeg =
        passing
            ? passingHoodAngleMap.get(lookaheadShooterToTargetDistance).getDegrees()
            : hoodAngleMap.get(lookaheadShooterToTargetDistance).getDegrees();
    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngleDeg;
    double hoodVelocityDegPerSec =
        hoodAngleFilter.calculate((hoodAngleDeg - lastHoodAngle) / Constants.loopPeriodSecs);
    lastHoodAngle = hoodAngleDeg;
    double driveVelocity =
        driveAngleFilter.calculate(
            driveAngle.minus(lastDriveAngle).getRadians() / Constants.loopPeriodSecs);
    lastDriveAngle = driveAngle;
    double flywheelSpeedRPS =
        applyFlywheelSpeedOffsets(
            passing
                ? passingFlywheelSpeedMap.get(lookaheadShooterToTargetDistance)
                : flywheelSpeedMap.get(lookaheadShooterToTargetDistance));

    latestParameters =
        new ShootingParameters(
            lookaheadShooterToTargetDistance >= (passing ? passingMinDistance : minDistance)
                && lookaheadShooterToTargetDistance <= (passing ? passingMaxDistance : maxDistance),
            driveAngle,
            driveVelocity,
            hoodAngleDeg,
            hoodVelocityDegPerSec,
            flywheelSpeedRPS,
            lookaheadShooterToTargetDistance,
            shooterToTargetDistance,
            timeOfFlight,
            passing);

    // Log calculated values
    Logger.recordOutput("ShotCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("ShotCalculator/ShooterToTargetDistance", lookaheadShooterToTargetDistance);

    return latestParameters;
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

  /** Adjusts the flywheel speed offset up or down the specified amount. */
  public void incrementFlywheelSpeedOffset(double incrementRPS) {
    flywheelSpeedOffsetRPS += incrementRPS;
  }

  /**
   * Adjusts the flywheel speed offset percentage, which is applied as a scalar to calculated target
   * speeds.
   *
   * <p>The {@code percent} value is a fractional scalar, where {@code 0.05} represents a +5%
   * increase and {@code -0.05} represents a -5% decrease in the calculated target speed.
   */
  public void incrementFlywheelSpeedOffsetPercent(double percent) {
    flywheelSpeedOffsetPercent += percent;
  }

  /**
   * Returns the Pose2d that correctly aims the robot at the goal for a given robot translation.
   *
   * @param robotTranslation The translation of the center of the robot.
   * @param forceBlue Always use the blue hub target.
   * @return The target pose for the aimed robot.
   */
  public static Pose2d getStationaryAimedPose(Translation2d robotTranslation, boolean forceBlue) {
    Translation2d target = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    if (!forceBlue) {
      target = AllianceFlipUtil.apply(target);
    }
    return new Pose2d(
        robotTranslation, getDriveAngleWithShooterOffset(robotTranslation.toPose2d(), target));
  }

  /** Convenience overload that always respects alliance color. */
  public static Pose2d getStationaryAimedPose(Translation2d robotTranslation) {
    return getStationaryAimedPose(robotTranslation, false);
  }
}
