// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SlewRateLimiter2d;
import frc.robot.util.TunableControls.ControlConstants;
import frc.robot.util.TunableControls.TunableControlConstants;
import frc.robot.util.TunableControls.TunablePIDController;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Default drive command to run that drives based on controller input */
public class DriveCommands extends Command {
  private static final String ZONE_AUTO_LOCK_DISABLED_KEY = "DriveCommands/ZoneAutoLockDisabled";
  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final SlewRateLimiter2d driveLimiter;

  // Alignment
  private static final ControlConstants TRENCH_TRANSLATION_BASE_CONSTANTS =
      new ControlConstants().withPID(8, 0, 0.05).withTolerance(0.05);
  private static final ControlConstants ROTATION_BASE_CONSTANTS =
      new ControlConstants()
          .withPID(10, 0, .5)
          .withTolerance(0.08)
          .withContinuous(-Math.PI, Math.PI);

  public static final TunableControlConstants TRENCH_TRANSLATION_CONSTANTS =
      new TunableControlConstants("Swerve/Trench Translation", TRENCH_TRANSLATION_BASE_CONSTANTS);
  public static final TunableControlConstants ROTATION_CONSTANTS =
      new TunableControlConstants("Swerve/Rotation", ROTATION_BASE_CONSTANTS);

  // Launch-while-driving tuning
  private static final LoggedTunableNumber driveLaunchToleranceDeg =
      new LoggedTunableNumber("DriveCommands/Launching/ToleranceDeg", 10.0);
  private static final LoggedTunableNumber driveLaunchMaxPolarVelocityRadPerSec =
      new LoggedTunableNumber("DriveCommands/Launching/MaxPolarVelocityRadPerSec", 0.5);

  // public static final LinearVelocity DEFAULT_DRIVE_SPEED = MetersPerSecond.of(3.2);
  // public static final AngularVelocity DEFAULT_ROT_SPEED = RotationsPerSecond.of(0.75);

  public static final LinearVelocity DEFAULT_DRIVE_SPEED = MetersPerSecond.of(4.5);
  public static final AngularVelocity DEFAULT_ROT_SPEED = RadiansPerSecond.of(12);

  public static final LinearVelocity SLOW_DRIVE_SPEED = MetersPerSecond.of(3);
  public static final AngularVelocity SLOW_ROT_SPEED = RotationsPerSecond.of(1);

  public static final LinearAcceleration MAX_TELEOP_ACCEL = MetersPerSecondPerSecond.of(11);

  private int flipFactor = 1; // 1 for normal, -1 for flipped
  private boolean headingLocked = false;
  @AutoLogOutput private Rotation2d targetHeading = Rotation2d.kZero;
  private DriveMode activeHeadingLockMode = null;

  private LinearVelocity maxDriveSpeed = DEFAULT_DRIVE_SPEED;
  private AngularVelocity maxRotSpeed = DEFAULT_ROT_SPEED;

  @AutoLogOutput
  private final Trigger inTrenchZoneTrigger = new Trigger(this::inTrenchZone).debounce(0.1);

  @AutoLogOutput
  private final Trigger inBumpZoneTrigger = new Trigger(this::inBumpZone).debounce(0.1);

  private final TunablePIDController trenchYController =
      new TunablePIDController(TRENCH_TRANSLATION_CONSTANTS);
  private final TunablePIDController rotationController =
      new TunablePIDController(ROTATION_CONSTANTS);

  @AutoLogOutput private DriveMode currentDriveMode = DriveMode.NORMAL;
  private boolean launchRequested = false;
  private boolean zoneAutoLockDisabled = true;
  private boolean driving = true;

  /** Creates a new DriveCommands. */
  public DriveCommands(Drive drive, CommandXboxController controller) {
    this.drive = drive;
    this.xSupplier = () -> -controller.getLeftY() * flipFactor;
    this.ySupplier = () -> -controller.getLeftX() * flipFactor;
    this.omegaSupplier = () -> -controller.getRightX();
    this.driveLimiter = new SlewRateLimiter2d(MAX_TELEOP_ACCEL.in(MetersPerSecondPerSecond));

    inTrenchZoneTrigger.onTrue(updateDriveMode(DriveMode.TRENCH_LOCK));
    inBumpZoneTrigger.onTrue(updateDriveMode(DriveMode.BUMP_LOCK));
    inTrenchZoneTrigger.or(inBumpZoneTrigger).onFalse(updateDriveMode(DriveMode.NORMAL));
    publishZoneAutoLockDisabled();
    addRequirements(drive);
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(x, y), ControllerConstants.CONTROLLER_DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  private boolean inTrenchZone() {
    Pose2d robotPose = drive.getPose();
    for (Translation2d[] zone : Constants.FieldConstants.TRENCH_ZONES) {
      if (robotPose.getX() >= zone[0].getX()
          && robotPose.getX() <= zone[1].getX()
          && robotPose.getY() >= zone[0].getY()
          && robotPose.getY() <= zone[1].getY()) {
        return true;
      }
    }
    return false;
  }

  private Distance getTrenchY() {
    Pose2d robotPose = drive.getPose();
    if (robotPose.getMeasureY().gte(Constants.FieldConstants.FIELD_WIDTH.div(2))) {
      return Constants.FieldConstants.FIELD_WIDTH.minus(Constants.FieldConstants.TRENCH_CENTER);
    }
    return Constants.FieldConstants.TRENCH_CENTER;
  }

  private Rotation2d getTrenchLockAngle() {
    for (int i = -90; i <= 180; i += 90) {
      if (Math.abs(MathUtil.inputModulus(drive.getRotation().getDegrees() - i, -180, 180)) <= 45) {
        return Rotation2d.fromDegrees(i);
      }
    }
    return Rotation2d.kZero;
  }

  private Rotation2d getBumpLockAngle() {
    for (int i = -135; i < 180; i += 90) {
      if (Math.abs(MathUtil.inputModulus(drive.getRotation().getDegrees() - i, -180, 180)) <= 45) {
        return Rotation2d.fromDegrees(i);
      }
    }
    return Rotation2d.kZero;
  }

  private boolean inBumpZone() {
    Pose2d robotPose = drive.getPose();
    for (Translation2d[] zone : Constants.FieldConstants.BUMP_ZONES) {
      if (robotPose.getX() >= zone[0].getX()
          && robotPose.getX() <= zone[1].getX()
          && robotPose.getY() >= zone[0].getY()
          && robotPose.getY() <= zone[1].getY()) {
        return true;
      }
    }
    return false;
  }

  private Command updateDriveMode(DriveMode driveMode) {
    return Commands.runOnce(
        () -> currentDriveMode = sanitizeRequestedDriveMode(driveMode, zoneAutoLockDisabled));
  }

  private static DriveMode sanitizeRequestedDriveMode(
      DriveMode requestedDriveMode, boolean zoneAutoLockDisabled) {
    if (zoneAutoLockDisabled
        && (requestedDriveMode == DriveMode.TRENCH_LOCK
            || requestedDriveMode == DriveMode.BUMP_LOCK)) {
      return DriveMode.NORMAL;
    }
    return requestedDriveMode;
  }

  private static DriveMode getZoneDriveMode(
      boolean zoneAutoLockDisabled, boolean inTrenchZone, boolean inBumpZone) {
    if (zoneAutoLockDisabled) {
      return DriveMode.NORMAL;
    }
    if (inTrenchZone) {
      return DriveMode.TRENCH_LOCK;
    }
    if (inBumpZone) {
      return DriveMode.BUMP_LOCK;
    }
    return DriveMode.NORMAL;
  }

  private static DriveMode resolveEffectiveMode(
      DriveMode currentDriveMode,
      boolean launchRequested,
      boolean zoneAutoLockDisabled,
      boolean inTrenchZone,
      boolean inBumpZone) {
    if (!launchRequested) {
      return currentDriveMode;
    }
    DriveMode zoneDriveMode = getZoneDriveMode(zoneAutoLockDisabled, inTrenchZone, inBumpZone);
    return zoneDriveMode != DriveMode.NORMAL ? zoneDriveMode : DriveMode.LAUNCH_LOCK;
  }

  private void publishZoneAutoLockDisabled() {
    SmartDashboard.putBoolean(ZONE_AUTO_LOCK_DISABLED_KEY, zoneAutoLockDisabled);
    Logger.recordOutput(ZONE_AUTO_LOCK_DISABLED_KEY, zoneAutoLockDisabled);
  }

  private void setHeadingLock(DriveMode driveMode, Rotation2d heading) {
    if (!headingLocked || activeHeadingLockMode != driveMode || !targetHeading.equals(heading)) {
      targetHeading = heading;
      headingLocked = true;
      activeHeadingLockMode = driveMode;
      rotationController.reset();
      rotationController.setSetpoint(heading.getRadians());
    }
  }

  private void clearHeadingLock() {
    headingLocked = false;
    activeHeadingLockMode = null;
  }

  public void driveFieldCentric(LinearVelocity xVel, LinearVelocity yVel, AngularVelocity omega) {
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, omega, drive.getRotation()));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flipFactor =
        DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
            ? -1
            : 1;

    setDriveSpeed(DEFAULT_DRIVE_SPEED);
    setRotSpeed(DEFAULT_ROT_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    linearVelocity = linearVelocity.times(maxDriveSpeed.in(MetersPerSecond));
    linearVelocity = driveLimiter.calculate(linearVelocity);

    // Determine translation and target heading based on drive mode
    boolean driverCommanding = false;

    // Resolve effective mode: trench/bump zones override launch lock heading,
    // but ShotCalculator keeps running so the solution stays fresh
    DriveMode effectiveMode =
        resolveEffectiveMode(
            currentDriveMode, launchRequested, zoneAutoLockDisabled, inTrenchZone(), inBumpZone());

    // Keep shot solution fresh whenever the driver wants to launch,
    // even if trench/bump heading lock is active
    if (launchRequested) {
      ShotCalculator.getInstance().getParameters();
    }

    switch (effectiveMode) {
      case NORMAL:
        double omega =
            MathUtil.applyDeadband(
                omegaSupplier.getAsDouble(), ControllerConstants.CONTROLLER_DEADBAND);
        omega = Math.copySign(omega * omega, omega); // square for more precise rotation control

        clearHeadingLock();
        driverCommanding = true;
        driveFieldCentric(
            MetersPerSecond.of(linearVelocity.getX()),
            MetersPerSecond.of(linearVelocity.getY()),
            maxRotSpeed.times(omega));
        break;
      case TRENCH_LOCK:
        trenchYController.setSetpoint(getTrenchY().in(Meters));
        double trenchY = trenchYController.calculate(drive.getPose().getY());
        if (trenchYController.atSetpoint()) {
          trenchY = 0;
        }
        linearVelocity = new Translation2d(linearVelocity.getX(), trenchY);
        setHeadingLock(DriveMode.TRENCH_LOCK, getTrenchLockAngle());
        ShotCalculator.getInstance().dropHood();
        break;
      case BUMP_LOCK:
        setHeadingLock(DriveMode.BUMP_LOCK, getBumpLockAngle());
        break;
      case LAUNCH_LOCK:
        {
          final var params = ShotCalculator.getInstance().getParameters();

          // Limit linear velocity so the hub angle change over TOF stays within polar limit.
          // Logic ported from Mechanical Advantage's 2026
          // DriveCommands.joystickDriveWhileLaunching.
          if (!params.passing()) {
            double maxLinearVelocityMagnitude = Double.POSITIVE_INFINITY;
            double robotAngle =
                Math.abs(
                    AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
                        .minus(drive.getPose().getTranslation())
                        .getAngle()
                        .minus(linearVelocity.getAngle())
                        .getRadians());
            double robotHubDistance = params.distanceNoLookahead();
            double hubAngle =
                driveLaunchMaxPolarVelocityRadPerSec.get()
                    * ShotCalculator.getInstance().getNaiveTOF(robotHubDistance);
            double lookaheadAngle = Math.PI - robotAngle - hubAngle;

            // Calculate limit if triangle is valid (otherwise no limit)
            if (lookaheadAngle > 0) {
              double robotLookaheadDistance =
                  robotHubDistance * Math.sin(hubAngle) / Math.sin(lookaheadAngle);
              maxLinearVelocityMagnitude =
                  robotLookaheadDistance
                      / ShotCalculator.getInstance().getNaiveTOF(robotHubDistance);
            }

            // Apply limit to velocity
            if (linearVelocity.getNorm() > maxLinearVelocityMagnitude) {
              linearVelocity =
                  linearVelocity.times(maxLinearVelocityMagnitude / linearVelocity.getNorm());
            }
          }

          // Set the target heading from ShotCalculator — the shared PID block below will drive it
          setHeadingLock(DriveMode.LAUNCH_LOCK, params.driveAngle());

          Logger.recordOutput(
              "DriveCommands/Launching/ErrorPosition",
              params.driveAngle().minus(drive.getRotation()));
          Logger.recordOutput("DriveCommands/Launching/SetpointPosition", params.driveAngle());
          Logger.recordOutput("DriveCommands/Launching/MeasuredPosition", drive.getRotation());
          break;
        }
    }

    if (!driving) {
      linearVelocity = new Translation2d();
    }

    // Apply heading lock PID if not directly commanding rotation
    if (!driverCommanding && headingLocked) {
      rotationController.setSetpoint(targetHeading.getRadians());
      double headingCorrection = rotationController.calculate(drive.getRotation().getRadians());
      if (rotationController.atSetpoint()) {
        headingCorrection = 0;
      }
      headingCorrection =
          MathUtil.clamp(
              headingCorrection,
              -maxRotSpeed.in(RadiansPerSecond),
              maxRotSpeed.in(RadiansPerSecond));
      driveFieldCentric(
          MetersPerSecond.of(linearVelocity.getX()),
          MetersPerSecond.of(linearVelocity.getY()),
          RadiansPerSecond.of(headingCorrection));
    }
  }

  private void setDriveSpeed(LinearVelocity speed) {
    maxDriveSpeed = speed;
  }

  private void setRotSpeed(AngularVelocity speed) {
    maxRotSpeed = speed;
  }

  private void resetHeadingGoal(Rotation2d heading) {
    targetHeading = heading;
    headingLocked = true;
    activeHeadingLockMode = null;
    rotationController.reset();
    rotationController.setSetpoint(heading.getRadians());
  }

  public Command resetGyroCommand() {
    return Commands.runOnce(
            () -> {
              Rotation2d allianceForward = AllianceFlipUtil.apply(Rotation2d.kZero);
              drive.setPose(new Pose2d(drive.getPose().getTranslation(), allianceForward));
              resetHeadingGoal(allianceForward);
            })
        .withName("ResetGyro");
  }

  public Command slowDownCommand() {
    return Commands.startEnd(
        () -> {
          setDriveSpeed(SLOW_DRIVE_SPEED);
          setRotSpeed(SLOW_ROT_SPEED);
        },
        () -> {
          setDriveSpeed(DEFAULT_DRIVE_SPEED);
          setRotSpeed(DEFAULT_ROT_SPEED);
        });
  }

  /** Returns true when the drivetrain is rotationally aligned to the hub shot target. */
  @AutoLogOutput(key = "DriveCommands/HubRotationAligned")
  public boolean atLaunchGoal() {
    var shotParams = ShotCalculator.getInstance().getParameters();
    return DriverStation.isEnabled()
        && shotParams != null
        && shotParams.isValid()
        && Math.abs(drive.getRotation().minus(shotParams.driveAngle()).getRadians())
            <= edu.wpi.first.math.util.Units.degreesToRadians(driveLaunchToleranceDeg.get());
  }

  /**
   * Returns a command that engages LAUNCH_LOCK drive mode while held and reverts to NORMAL on
   * release.
   */
  public Command launchModeCommand() {
    return Commands.startEnd(() -> launchRequested = true, () -> launchRequested = false, drive);
  }

  public Command launchModeAndStopCommand() {
    return Commands.sequence(
        Commands.run(drive::stop, drive).withTimeout(.05), launchModeCommand());
  }

  public void setZoneAutoLockDisabled(boolean zoneAutoLockDisabled) {
    if (this.zoneAutoLockDisabled == zoneAutoLockDisabled) {
      return;
    }
    this.zoneAutoLockDisabled = zoneAutoLockDisabled;
    currentDriveMode = getZoneDriveMode(this.zoneAutoLockDisabled, inTrenchZone(), inBumpZone());
    if (this.zoneAutoLockDisabled
        && (activeHeadingLockMode == DriveMode.TRENCH_LOCK
            || activeHeadingLockMode == DriveMode.BUMP_LOCK)) {
      clearHeadingLock();
    }
    publishZoneAutoLockDisabled();
  }

  public boolean isZoneAutoLockDisabled() {
    return zoneAutoLockDisabled;
  }

  public void syncDashboardOverrides() {
    setZoneAutoLockDisabled(
        SmartDashboard.getBoolean(ZONE_AUTO_LOCK_DISABLED_KEY, isZoneAutoLockDisabled()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private enum DriveMode {
    NORMAL,
    TRENCH_LOCK,
    BUMP_LOCK,
    LAUNCH_LOCK
  }
}
