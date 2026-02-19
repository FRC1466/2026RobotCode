// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final LoggedTunableNumber ANGLE_KP =
      new LoggedTunableNumber("DriveCommands/AngleKP", 5.0);
  private static final LoggedTunableNumber ANGLE_KD =
      new LoggedTunableNumber("DriveCommands/AngleKD", 0.4);
  private static final LoggedTunableNumber ANGLE_MAX_VELOCITY =
      new LoggedTunableNumber("DriveCommands/AngleMaxVelocity", 8.0);
  private static final LoggedTunableNumber ANGLE_MAX_ACCELERATION =
      new LoggedTunableNumber("DriveCommands/AngleMaxAcceleration", 20.0);
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private static final LoggedTunableNumber LAUNCH_KP =
      new LoggedTunableNumber("DriveCommands/Launching/kP", 4.0);
  private static final LoggedTunableNumber LAUNCH_KD =
      new LoggedTunableNumber("DriveCommands/Launching/kD", 0.2);
  private static final LoggedTunableNumber LAUNCH_TOLERANCE_DEG =
      new LoggedTunableNumber("DriveCommands/Launching/ToleranceDeg", 10.0);
  private static final LoggedTunableNumber LAUNCH_MAX_POLAR_VEL =
      new LoggedTunableNumber("DriveCommands/Launching/MaxPolarVelocityRadPerSec", 0.5);

  // Trench auto-alignment constants
  // X-axis lookahead: how far (in meters) in front of the trench opening to begin engaging
  private static final LoggedTunableNumber TRENCH_APPROACH_X_MARGIN =
      new LoggedTunableNumber("DriveCommands/Trench/ApproachXMarginMeters", 1.5);
  // Y PID for centering the robot through the trench opening
  private static final LoggedTunableNumber TRENCH_Y_KP =
      new LoggedTunableNumber("DriveCommands/Trench/YKP", 4.0);
  private static final LoggedTunableNumber TRENCH_Y_KD =
      new LoggedTunableNumber("DriveCommands/Trench/YKD", 0.2);
  // Angle PID (reuses ANGLE_* tunable numbers above for the snap controller)
  // Max Y correction speed as a fraction of max linear speed
  private static final LoggedTunableNumber TRENCH_MAX_Y_CORRECTION =
      new LoggedTunableNumber("DriveCommands/Trench/MaxYCorrectionFraction", 0.6);

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Returns the nearest cardinal angle (0, 90, 180, 270 degrees) to the given rotation, expressed
   * in the same frame.
   */
  private static Rotation2d nearestCardinal(Rotation2d rotation) {
    double deg = rotation.getDegrees();
    // Round to nearest multiple of 90
    double snapped = Math.round(deg / 90.0) * 90.0;
    return Rotation2d.fromDegrees(snapped);
  }

  /**
   * Returns the target Y coordinate (in blue-origin field coordinates) to center the robot through
   * the trench it is approaching, or {@code Double.NaN} if not approaching either trench.
   *
   * <p>The robot is considered to be approaching a trench when:
   *
   * <ul>
   *   <li>Its field-relative Y (blue-origin) places it within the lateral span of either the right
   *       or left trench open region, AND
   *   <li>Its X is within {@code TRENCH_APPROACH_X_MARGIN} meters of the trench's hub-side mouth.
   * </ul>
   */
  private static double getTrenchTargetY(Pose2d pose) {
    // All trench geometry is defined in blue-origin coordinates; flip if red.
    double robotX = AllianceFlipUtil.applyX(pose.getX());
    double robotY = AllianceFlipUtil.applyY(pose.getY());

    double hubX = FieldConstants.LinesVertical.hubCenter;
    double approachMargin = TRENCH_APPROACH_X_MARGIN.get();

    // Right trench: low Y (Y between rightTrenchOpenEnd=0 and rightBumpStart)
    double rightTrenchYMin = FieldConstants.LinesHorizontal.rightTrenchOpenEnd;
    double rightTrenchYMax = FieldConstants.LinesHorizontal.rightBumpStart;
    double rightTrenchCenterY = FieldConstants.RightTrench.openingWidth / 2.0;
    boolean inRightTrenchY = robotY >= rightTrenchYMin && robotY <= rightTrenchYMax;
    boolean nearRightTrenchX = robotX >= hubX - approachMargin && robotX <= hubX + approachMargin;

    if (inRightTrenchY && nearRightTrenchX) {
      return AllianceFlipUtil.applyY(rightTrenchCenterY);
    }

    // Left trench: high Y (Y between leftBumpStart and fieldWidth)
    double leftTrenchYMin = FieldConstants.LinesHorizontal.leftBumpStart;
    double leftTrenchYMax = FieldConstants.LinesHorizontal.leftTrenchOpenStart;
    double leftTrenchCenterY =
        FieldConstants.fieldWidth - FieldConstants.LeftTrench.openingWidth / 2.0;
    boolean inLeftTrenchY = robotY >= leftTrenchYMin && robotY <= leftTrenchYMax;
    boolean nearLeftTrenchX = robotX >= hubX - approachMargin && robotX <= hubX + approachMargin;

    if (inLeftTrenchY && nearLeftTrenchX) {
      return AllianceFlipUtil.applyY(leftTrenchCenterY);
    }

    return Double.NaN;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   *
   * <p>When the robot is approaching a trench opening (right or left, alliance-relative), this
   * command automatically:
   *
   * <ul>
   *   <li>Snaps the heading to the nearest cardinal angle (0°, 90°, 180°, 270°) via PID, and
   *   <li>Controls field-relative Y to center through the trench opening via PD feedback,
   * </ul>
   *
   * while leaving the driver's X input (forward/back through the trench) unmodified.
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    // Angle snap controller (reuses ANGLE tunable numbers)
    ProfiledPIDController trenchAngleController =
        new ProfiledPIDController(
            ANGLE_KP.get(),
            0.0,
            ANGLE_KD.get(),
            new TrapezoidProfile.Constraints(
                ANGLE_MAX_VELOCITY.get(), ANGLE_MAX_ACCELERATION.get()));
    trenchAngleController.enableContinuousInput(-Math.PI, Math.PI);

    // PD state for the trench Y controller
    final double[] trenchYErrorPrev = {0.0};

    return Commands.run(
            () -> {
              // Update trench angle controller gains if tuned
              if (ANGLE_KP.hasChanged(ANGLE_KP.hashCode())
                  || ANGLE_KD.hasChanged(ANGLE_KD.hashCode())
                  || ANGLE_MAX_VELOCITY.hasChanged(ANGLE_MAX_VELOCITY.hashCode())
                  || ANGLE_MAX_ACCELERATION.hasChanged(ANGLE_MAX_ACCELERATION.hashCode())) {
                trenchAngleController.setP(ANGLE_KP.get());
                trenchAngleController.setD(ANGLE_KD.get());
                trenchAngleController.setConstraints(
                    new TrapezoidProfile.Constraints(
                        ANGLE_MAX_VELOCITY.get(), ANGLE_MAX_ACCELERATION.get()));
              }

              // Get linear velocity from joysticks
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Apply rotation deadband and squaring
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
              omega = Math.copySign(omega * omega, omega);

              // Check whether the robot is approaching a trench
              Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
              double trenchTargetY = getTrenchTargetY(currentPose);
              boolean inTrenchMode = !Double.isNaN(trenchTargetY);

              if (inTrenchMode) {
                // --- Heading: snap to nearest cardinal via profiled PID ---
                Rotation2d snapTarget = nearestCardinal(currentPose.getRotation());
                omega =
                    trenchAngleController.calculate(
                        currentPose.getRotation().getRadians(), snapTarget.getRadians());

                // --- Y: PD controller toward trench center; X remains driver-controlled ---
                double yError = trenchTargetY - currentPose.getY();
                double yErrorDot = (yError - trenchYErrorPrev[0]) / Constants.loopPeriodSecs;
                trenchYErrorPrev[0] = yError;
                double yCorrection = TRENCH_Y_KP.get() * yError + TRENCH_Y_KD.get() * yErrorDot;
                double maxYCorr =
                    TRENCH_MAX_Y_CORRECTION.get() * drive.getMaxLinearSpeedMetersPerSec();
                yCorrection = MathUtil.clamp(yCorrection, -maxYCorr, maxYCorr);

                // Replace the joystick Y fraction with the PD output (already in m/s)
                double xSpeed = linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec();

                Logger.recordOutput("DriveCommands/Trench/Active", true);
                Logger.recordOutput("DriveCommands/Trench/SnapTargetDeg", snapTarget.getDegrees());
                Logger.recordOutput("DriveCommands/Trench/YError", yError);
                Logger.recordOutput("DriveCommands/Trench/YCorrection", yCorrection);

                boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed,
                        yCorrection,
                        omega,
                        isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
              } else {
                // --- Normal drive ---
                trenchYErrorPrev[0] = 0.0;
                trenchAngleController.reset(currentPose.getRotation().getRadians());

                Logger.recordOutput("DriveCommands/Trench/Active", false);

                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega * drive.getMaxAngularSpeedRadPerSec());
                boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
              }
            },
            drive)
        .beforeStarting(
            () ->
                trenchAngleController.reset(
                    RobotState.getInstance().getEstimatedPose().getRotation().getRadians()));
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP.get(),
            0.0,
            ANGLE_KD.get(),
            new TrapezoidProfile.Constraints(
                ANGLE_MAX_VELOCITY.get(), ANGLE_MAX_ACCELERATION.get()));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              if (ANGLE_KP.hasChanged(ANGLE_KP.hashCode())
                  || ANGLE_KD.hasChanged(ANGLE_KD.hashCode())
                  || ANGLE_MAX_VELOCITY.hasChanged(ANGLE_MAX_VELOCITY.hashCode())
                  || ANGLE_MAX_ACCELERATION.hasChanged(ANGLE_MAX_ACCELERATION.hashCode())) {
                angleController.setP(ANGLE_KP.get());
                angleController.setD(ANGLE_KD.get());
                angleController.setConstraints(
                    new TrapezoidProfile.Constraints(
                        ANGLE_MAX_VELOCITY.get(), ANGLE_MAX_ACCELERATION.get()));
              }
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /** Returns true when the robot heading is within tolerance of the launch target angle. */
  public static boolean atLaunchGoal() {
    return DriverStation.isEnabled()
        && Math.abs(
                RobotState.getInstance()
                    .getRotation()
                    .minus(ShotCalculator.getInstance().getParameters().driveAngle())
                    .getRadians())
            <= Units.degreesToRadians(LAUNCH_TOLERANCE_DEG.get());
  }

  /**
   * Field relative drive command that auto-aims at the hub while the driver controls translation.
   * Limits linear velocity to keep the note's projected landing point within an acceptable polar
   * velocity around the hub.
   */
  public static Command joystickDriveWhileLaunching(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return Commands.run(
        () -> {
          // Run PD controller on heading
          final var parameters = ShotCalculator.getInstance().getParameters();
          double omegaOutput =
              parameters.driveVelocity()
                  + (parameters
                          .driveAngle()
                          .minus(RobotState.getInstance().getRotation())
                          .getRadians()
                      * LAUNCH_KP.get())
                  + ((parameters.driveVelocity()
                          - RobotState.getInstance().getFieldVelocity().omegaRadiansPerSecond)
                      * LAUNCH_KD.get());

          // Calculate field-relative linear velocity from joysticks
          Translation2d fieldRelativeLinearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                  .times(drive.getMaxLinearSpeedMetersPerSec());
          if (AllianceFlipUtil.shouldFlip()) {
            fieldRelativeLinearVelocity = fieldRelativeLinearVelocity.times(-1.0);
          }

          // Calculate max linear velocity magnitude based on the max polar velocity around the hub
          double maxLinearVelocityMagnitude = Double.POSITIVE_INFINITY;
          if (!parameters.passing()) {
            double robotAngle =
                Math.abs(
                    AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
                        .minus(RobotState.getInstance().getEstimatedPose().getTranslation())
                        .getAngle()
                        .minus(fieldRelativeLinearVelocity.getAngle())
                        .getRadians());
            double robotHubDistance = parameters.distanceNoLookahead();
            double hubAngle =
                LAUNCH_MAX_POLAR_VEL.get()
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
          }

          // Apply limit to velocity
          if (fieldRelativeLinearVelocity.getNorm() > maxLinearVelocityMagnitude) {
            fieldRelativeLinearVelocity =
                fieldRelativeLinearVelocity.times(
                    maxLinearVelocityMagnitude / fieldRelativeLinearVelocity.getNorm());
          }

          // Apply chassis speeds
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  fieldRelativeLinearVelocity.getX(),
                  fieldRelativeLinearVelocity.getY(),
                  omegaOutput,
                  RobotState.getInstance().getRotation()));

          // Log data
          Logger.recordOutput(
              "DriveCommands/Launching/ErrorPosition",
              parameters.driveAngle().minus(RobotState.getInstance().getRotation()));
          Logger.recordOutput(
              "DriveCommands/Launching/ErrorVelocityRadPerSec",
              parameters.driveVelocity()
                  - RobotState.getInstance().getFieldVelocity().omegaRadiansPerSecond);
          Logger.recordOutput(
              "DriveCommands/Launching/MeasuredPosition", RobotState.getInstance().getRotation());
          Logger.recordOutput(
              "DriveCommands/Launching/MeasuredVelocityRadPerSec",
              RobotState.getInstance().getFieldVelocity().omegaRadiansPerSecond);
          Logger.recordOutput("DriveCommands/Launching/SetpointPosition", parameters.driveAngle());
          Logger.recordOutput(
              "DriveCommands/Launching/SetpointVelocityRadPerSec", parameters.driveVelocity());
        },
        drive);
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
