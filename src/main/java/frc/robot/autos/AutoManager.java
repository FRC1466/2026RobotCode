// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.RobotContainer;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Owns the B-Line path follower, the auto chooser, and event trigger registration.
 *
 * <p>AutoRoutines calls {@link #followWithReset} for the first path in a routine and {@link
 * #follow} for every subsequent path. This keeps the drive's pose-reset behavior explicit and
 * per-routine.
 *
 * <h2>Authoring autos</h2>
 *
 * <b>GUI workflow (BLine-GUI):</b><br>
 * Open BLine-GUI, design your path on the field, and export. The GUI writes JSON files to {@code
 * deploy/autos/paths/} and updates {@code deploy/autos/config.json}. In Java, load the path with
 * {@code new Path("filename")} (no extension) and build a command with {@link #follow} or {@link
 * #followWithReset}.
 *
 * <p>Example: {@code manager.followWithReset(new Path("my_path"))}
 *
 * <p><b>Code-only workflow:</b><br>
 * Construct a {@link Path} directly from waypoints:
 *
 * <pre>{@code
 * Path p = new Path(
 *     new Path.Waypoint(new Pose2d(1.0, 4.0, Rotation2d.kZero)),
 *     new Path.TranslationTarget(3.0, 4.0),
 *     new Path.Waypoint(new Pose2d(5.0, 4.0, Rotation2d.fromDegrees(180))));
 * Command cmd = manager.followWithReset(p);
 * }</pre>
 *
 * <b>Event triggers (GUI paths):</b><br>
 * In BLine-GUI, add an EventTrigger element and set its {@code lib_key} to one of the keys
 * registered in {@link #registerEventTriggers}. The corresponding command fires when the robot
 * reaches that point in the path.
 */
public final class AutoManager {

  // B-Line PID gains derived from this robot's existing Choreo feedback controllers (P=7 for
  // x/y/heading). Cross-track starts conservative; increase if lateral path deviation is
  // noticeable during testing.
  private static final double TRANSLATION_KP = 7.0;
  private static final double ROTATION_KP = 7.0;
  private static final double CROSS_TRACK_KP = 3.0;

  private final Drive drive;
  private final FollowPath.Builder pathBuilder;
  private final SendableChooser<Command> chooser = new SendableChooser<>();
  private final Field2d startPoseField = new Field2d();
  private final Map<String, Pose2d> autoStartPoses = new HashMap<>();

  public AutoManager(RobotContainer rc) {
    this.drive = rc.getDrive();

    // 2026 REEFSCAPE uses rotational field symmetry (180° rotation about field center).
    FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;

    // Global path constraints. These values are also written to deploy/autos/config.json so
    // BLine-GUI uses the same limits when you preview paths. Robot max speed is 4.56 m/s;
    // 4.0 m/s leaves a reliability margin.
    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(
            4.0, // maxVelocityMetersPerSec
            3.0, // maxAccelerationMetersPerSec2
            360.0, // maxVelocityDegPerSec
            720.0, // maxAccelerationDegPerSec2
            0.05, // endTranslationToleranceMeters
            2.0, // endRotationToleranceDeg
            0.3 // intermediateHandoffRadiusMeters
            ));

    pathBuilder =
        new FollowPath.Builder(
                rc.getDrive(),
                rc.getDrive()::getPose,
                rc.getDrive()::getChassisSpeeds,
                rc.getDrive()::runVelocity,
                new PIDController(TRANSLATION_KP, 0.0, 0.0),
                new PIDController(ROTATION_KP, 0.0, 0.0),
                new PIDController(CROSS_TRACK_KP, 0.0, 0.0))
            .withDefaultShouldFlip();

    // All event trigger keys must be registered before any FollowPath command is ever scheduled.
    // The string keys here must exactly match the "lib_key" field in GUI-exported path JSON files.
    // Add new entries here when you add event triggers to GUI paths.
    registerEventTriggers(rc);

    AutoActions actions = new AutoActions(rc);
    AutoRoutines routines = new AutoRoutines(this, actions, rc);

    Pose2d driveBackStart = new Pose2d(3.6, 4.02, Rotation2d.kZero);
    Pose2d driveLeftStart = new Pose2d(3.38, 6.53, Rotation2d.fromRadians(-1.1104));

    chooser.setDefaultOption("None", Commands.none());
    addOption("Drive Back Preload Auto", routines.driveBackPreload(), driveBackStart);
    addOption("Drive Left Preload Auto", routines.driveLeftPreload(), driveLeftStart);
    addOption("Left Preload Auto", routines.driveLeftPreload(), driveLeftStart);
    addOption("Outpost Auto", routines.outpostAuto(), null);
    addOption("Preload Then Outpost Auto", routines.preloadThenOutpostAuto(), null);
    addOption("Ground Auto", routines.groundAuto(), null);
    addOption("Preload Then Ground Auto", routines.preloadThenGroundAuto(), null);
    addOption("Double Ground Pickup Auto", routines.doubleGroundPickupAuto(), null);
    addOption("Single Ground Pickup Auto", routines.singleGroundPickupAuto(), null);
    addOption("One Dip Left Auto", routines.oneDipLeftAuto(), null);

    SmartDashboard.putData("Auto Chooser", chooser);
    SmartDashboard.putData("Auto Start Pose", startPoseField);

    // Mirrors the old Autos class: autonomous mode triggers the selected command.
    // Commands.defer defers chooser.getSelected() until autonomous actually starts so
    // late dashboard changes during pre-match are respected.
    // ProxyCommand runs the selected command without registering it as composed, so the same
    // pre-built command can be re-scheduled across multiple auto enables.
    RobotModeTriggers.autonomous().whileTrue(new ProxyCommand(chooser::getSelected));
  }

  /**
   * Builds a FollowPath command that resets odometry to the path's first waypoint on initialize.
   * Use this for the first path in every routine.
   */
  Command followWithReset(Path path) {
    return pathBuilder.withPoseReset(drive::setPose).build(path);
  }

  /**
   * Builds a FollowPath command without an odometry reset. Use this for every path after the first
   * in a sequence — odometry from the preceding path carries forward.
   */
  Command follow(Path path) {
    return pathBuilder.withPoseReset(pose -> {}).build(path);
  }

  /** Update SmartDashboard and log outputs. Call from RobotContainer.updateDashboardOutputs(). */
  public void updateDashboardOutputs() {
    Command selected = chooser.getSelected();
    if (selected == null) return;
    String name = selected.getName();
    Logger.recordOutput("Auto/SelectedCommandName", name);

    Pose2d startPose = autoStartPoses.get(name);
    if (startPose != null) {
      boolean isRed =
          DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);
      startPoseField.setRobotPose(isRed ? FlippingUtil.flipFieldPose(startPose) : startPose);
    }
  }

  private void addOption(String name, Command command, Pose2d startPose) {
    chooser.addOption(name, command);
    if (startPose != null) autoStartPoses.put(name, startPose);
  }

  private void registerEventTriggers(RobotContainer rc) {
    FollowPath.registerEventTrigger("intake_deploy", rc.getIntake().deployCommand());
    FollowPath.registerEventTrigger("intake_run", rc.getIntake().runCommand());
    FollowPath.registerEventTrigger("intake_stop", rc.getIntake().stopCommand());
    FollowPath.registerEventTrigger("intake_stow", rc.getIntake().stowCommand());
  }
}
