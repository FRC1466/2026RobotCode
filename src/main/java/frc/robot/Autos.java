// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.Choreographer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.AllianceFlipUtil;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized autonomous routine factory for all auto modes.
 *
 * <h2>Architecture Overview</h2>
 *
 * This class uses <b>ChoreoLib's trigger-based AutoRoutine API</b> to compose autonomous routines
 * that run Choreo trajectories alongside robot actions (intaking, shooting, etc.) in a reactive,
 * event-driven manner.
 *
 * <h2>Key Concepts</h2>
 *
 * <h3>AutoFactory</h3>
 *
 * Created once in the constructor. Provides the bridge between Choreo trajectory files ({@code
 * .chor} in {@code src/main/deploy/choreo/}) and the drive subsystem's path-following controller
 * ({@link Drive#followTrajectory}). Alliance flipping is handled automatically.
 *
 * <h3>AutoRoutine</h3>
 *
 * Each auto mode is an {@link AutoRoutine} — a self-contained event loop with its own set of {@link
 * edu.wpi.first.wpilibj2.command.button.Trigger Trigger} bindings. Routines are registered with the
 * {@link AutoChooser} via method references (e.g. {@code this::depotAuto}) so they are lazily
 * constructed only when selected.
 *
 * <h3>AutoTrajectory</h3>
 *
 * <h3>Strategy 1: Split Waypoints (single .chor file, no branching)</h3>
 *
 * Use this when the auto is a fixed linear sequence. In the Choreo GUI, enable the <b>"Split"</b>
 * checkbox on any waypoint that has a stop-point. This produces indexed segments from a single
 * trajectory file that you load by split index:
 *
 * <pre>{@code
 * // In Choreo: depotAuto.chor with waypoints 0–1–2, split enabled on waypoint 1
 * // Produces two segments: index 0 (waypoints 0→1) and index 1 (waypoints 1→2)
 *
 * AutoTrajectory toDepot   = routine.trajectory("depotAuto", 0);
 * AutoTrajectory toScore   = routine.trajectory("depotAuto", 1);
 *
 * routine.active().onTrue(
 *     Commands.sequence(toDepot.resetOdometry(), toDepot.cmd()));
 * toDepot.chain(toScore);  // equivalent to: toDepot.done().onTrue(toScore.cmd())
 * }</pre>
 *
 * <b>Pros:</b> Single file, easy to maintain, waypoints share constraints. <br>
 * <b>Cons:</b> Cannot branch — the segment sequence is fixed at build time.
 *
 * <h3>Strategy 2: Separate Trajectory Files (branching autos)</h3>
 *
 * Use this when the robot must decide at runtime which path to take next (e.g. "did we pick up a
 * game piece?"). Create separate {@code .chor} files (or separate trajectories within one project)
 * for each segment between decision points:
 *
 * <pre>{@code
 * AutoTrajectory startToPickup = routine.trajectory("startToPickup");
 * AutoTrajectory pickupToScore = routine.trajectory("pickupToScore");
 * AutoTrajectory pickupToDepot = routine.trajectory("pickupToDepot");
 *
 * routine.active().onTrue(
 *     Commands.sequence(startToPickup.resetOdometry(), startToPickup.cmd()));
 *
 * // Branch based on game piece state
 * startToPickup.done().and(this::hasGamePiece).onTrue(pickupToScore.cmd());
 * startToPickup.done().and(() -> !hasGamePiece()).onTrue(pickupToDepot.cmd());
 * }</pre>
 *
 * <b>Pros:</b> Full runtime branching, reusable segments across routines. <br>
 * <b>Cons:</b> More files to manage; endpoints must match between connected segments.
 *
 * <h2>Running Commands During Trajectories</h2>
 *
 * The trigger API lets you run commands <b>concurrently</b> with trajectory following without
 * composing them into a single sequence. The drive subsystem is only required by the trajectory
 * command itself, so other subsystem commands run freely in parallel.
 *
 * <pre>{@code
 * // ── Lifecycle triggers ──────────────────────────────────────────
 * // Run intake the ENTIRE time a trajectory is active
 * pickupTraj.active().whileTrue(choreographer.setGoalCommand(Goal.INTAKE));
 * private void registerRoutine(
 * String name,
 * Supplier<AutoRoutine> routineSupplier,
 * Supplier<Optional<Pose2d>> startPoseSupplier) {
 * autoChooser.addRoutine(name, routineSupplier);
 * autoStartPoseSuppliers.put(name, startPoseSupplier);
 * }
 *
 * // ── Event marker triggers ───────────────────────────────────────
 * // In Choreo, place an event marker named "startIntake" on the path.
 * // This fires once when the robot reaches that timestamp:
 * pickupTraj.atTime("startIntake").onTrue(choreographer.setGoalCommand(Goal.INTAKE));
 *
 * // ── Time-based triggers ─────────────────────────────────────────
 * // Start spinning up the shooter 0.5s before the trajectory ends
 * scoreTraj.atTimeBeforeEnd(0.5).onTrue(choreographer.setGoalCommand(Goal.SCORE_HUB));
 *
 * // ── Done triggers ───────────────────────────────────────────────
 * // When trajectory finishes, shoot then chain to next path
 * scoreTraj.done().onTrue(
 *     Commands.sequence(
 *         choreographer.setGoalCommand(Goal.SCORE_HUB),
 *         Commands.waitUntil(choreographer::isReadyToShoot),
 *         Commands.waitSeconds(0.3),  // let the shot leave
 *         choreographer.setGoalCommand(Goal.IDLE)));
 * }</pre>
 *
 * <h2>PathPlanner On-The-Fly Paths</h2>
 *
 * Use PathPlanner's {@code AutoBuilder.pathfindToPose()} for segments that cannot be pre-planned
 * (e.g. driving to an arbitrary field position after a miss). Since AutoBuilder is already
 * configured in {@link Drive}, you can schedule pathfinding commands from any trigger:
 *
 * <pre>{@code
 * import com.pathplanner.lib.auto.AutoBuilder;
 *
 * // After finishing a Choreo trajectory, pathfind to a recovery pose
 * scoreTraj.done().and(() -> !hasGamePiece()).onTrue(
 *     AutoBuilder.pathfindToPose(recoveryPose, new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 *
 *         Math.PI)));
 * }</pre>
 *
 * <b>When to use on-the-fly vs. Choreo:</b>
 *
 * <ul>
 *   <li><b>Choreo</b> — pre-planned, time-optimal trajectories. Best for known fixed paths where
 *       trajectory quality matters (scoring runs, fast cycles).
 *   <li><b>PathPlanner on-the-fly</b> — obstacle-aware pathfinding from the robot's current pose to
 *       any target. Best for recovery, repositioning, or dynamic targets. Slower than Choreo
 *       trajectories because they are generated at runtime.
 * </ul>
 *
 * <h2>Template for a New Auto Routine</h2>
 *
 * <pre>{@code
 * public AutoRoutine myNewAuto() {
 *   AutoRoutine routine = autoFactory.newRoutine("My New Auto");
 *
 *   // 1. Load trajectories (split or separate files)
 *   AutoTrajectory seg1 = routine.trajectory("myAuto", 0);
 *   AutoTrajectory seg2 = routine.trajectory("myAuto", 1);
 *
 *   // 2. Start: reset odometry then follow first segment
 *   routine.active().onTrue(
 *       Commands.sequence(seg1.resetOdometry(), seg1.cmd()));
 *
 *   // 3. Wire up concurrent actions via triggers
 *   seg1.active().whileTrue(choreographer.setGoalCommand(Goal.INTAKE));
 *   seg1.atTimeBeforeEnd(0.5).onTrue(choreographer.setGoalCommand(Goal.SCORE_HUB));
 *
 *   // 4. Chain segments
 *   seg1.done().onTrue(seg2.cmd());
 *
 *   // 5. Final action
 *   seg2.done().onTrue(choreographer.setGoalCommand(Goal.IDLE));
 *
 *   return routine;
 * }
 * }</pre>
 *
 * <h2>Common Pitfalls</h2>
 *
 * <ul>
 *   <li>{@code done()} is true for only <b>one cycle</b> — use {@code .onTrue()}, not {@code
 *       .whileTrue()}.
 *   <li>Always call {@code resetOdometry()} on the <b>first</b> trajectory before {@code cmd()}.
 *   <li>Don't mix {@code autoFactory.trajectoryCmd()} (command-composition style) with {@code
 *       routine.trajectory()} (trigger style) in the same routine.
 *   <li>Each {@link AutoRoutine} has its own event loop — triggers from one routine do not leak
 *       into another.
 *   <li>The Choreographer's {@code setGoalCommand()} returns an instant command. Pair it with
 *       {@code Commands.waitUntil()} or {@code Commands.waitSeconds()} when you need to wait for a
 *       mechanism to reach its goal before proceeding.
 * </ul>
 *
 * @see <a href="https://choreo.autos/choreolib/auto-routines">ChoreoLib Auto Routine Docs</a>
 */
public class Autos {
  private final Drive drive;
  private final RobotContainer robotContainer;
  private final Choreographer choreographer;
  private final Intake intake;
  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;
  private final Map<String, Supplier<Optional<Pose2d>>> autoStartPoseSuppliers;
  private final Map<String, Supplier<Optional<Pose2d>>> autoStartPoseSuppliersByCommandName;
  private final edu.wpi.first.wpilibj.smartdashboard.Field2d autoStartField;

  public Autos(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    this.drive = robotContainer.getDrive();
    this.choreographer = robotContainer.getChoreographer();
    this.intake = robotContainer.getIntake();

    autoFactory =
        new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectory, true, drive);

    autoChooser = new AutoChooser();
    autoStartPoseSuppliers = new java.util.HashMap<>();
    autoStartPoseSuppliersByCommandName = new java.util.HashMap<>();
    autoStartPoseSuppliers.put(autoChooser.getDefaultName(), Optional::<Pose2d>empty);
    autoStartPoseSuppliersByCommandName.put(autoChooser.getDefaultName(), Optional::<Pose2d>empty);

    // Register all autonomous routines here.
    // Use one registration path so chooser entries and start-pose metadata stay in sync.
    registerRoutine(
        "Depot Auto",
        this::depotAuto,
        () -> startPoseOf(this::depotAuto, routine -> routine.trajectory("DepotAuto", 0)));
    registerRoutine(
        "Ground Depot Auto",
        this::groundDepotAuto,
        () ->
            startPoseOf(
                this::groundDepotAuto, routine -> routine.trajectory("DepotAutoGround", 0)));
    registerRoutine(
        "Ground Auto",
        this::groundAuto,
        () -> startPoseOf(this::groundAuto, routine -> routine.trajectory("GrabFromGround", 0)));
    registerRoutine(
        "Rush To Center Auto",
        this::rushToCenterAuto,
        () ->
            startPoseOf(this::rushToCenterAuto, routine -> routine.trajectory("RushToCenter", 0)));
    registerRoutine("Shoot Preload Auto", this::shootFromAnywhereAuto, Optional::<Pose2d>empty);
    registerRoutine(
        "Drive Back Preload Auto",
        this::driveBackPreloadAuto,
        () ->
            Optional.of(
                AllianceFlipUtil.apply(
                    new Pose2d(3.5, FieldConstants.fieldWidth / 2.0, Rotation2d.kZero))));

    autoStartField = new edu.wpi.first.wpilibj.smartdashboard.Field2d();
    SmartDashboard.putData("Auto Start Pose", autoStartField);

    SmartDashboard.putData("Auto Chooser", autoChooser);

    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  /**
   * Returns the {@link AutoChooser} to be placed on the dashboard for driver selection.
   *
   * @return the auto chooser widget
   */
  public AutoChooser getAutoChooser() {
    return autoChooser;
  }

  public void updateDashboardOutputs() {
    if (!DriverStation.isDisabled()) {
      return;
    }

    String selectedCommandName = autoChooser.selectedCommand().getName();
    Optional<Pose2d> startPose =
        autoStartPoseSuppliersByCommandName
            .getOrDefault(selectedCommandName, Optional::<Pose2d>empty)
            .get();

    startPose.ifPresentOrElse(
        autoStartField::setRobotPose, () -> autoStartField.setRobotPose(new Pose2d()));

    Logger.recordOutput("Autos/SelectedCommandName", selectedCommandName);
    Logger.recordOutput("Autos/HasStartPose", startPose.isPresent());
    Logger.recordOutput(
        "Autos/StartPose",
        startPose.map(pose -> new Pose2d[] {pose}).orElseGet(() -> new Pose2d[0]));
  }

  private void registerRoutine(
      String name,
      Supplier<AutoRoutine> routineSupplier,
      Supplier<Optional<Pose2d>> startPoseSupplier) {
    autoChooser.addRoutine(name, routineSupplier);
    autoStartPoseSuppliers.put(name, startPoseSupplier);
    autoStartPoseSuppliersByCommandName.put(name, startPoseSupplier);

    AutoRoutine previewRoutine = routineSupplier.get();
    autoStartPoseSuppliersByCommandName.put(previewRoutine.cmd().getName(), startPoseSupplier);
  }

  public AutoRoutine shootFromAnywhereAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Shoot From Anywhere");

    Supplier<Pose2d> targetPoseSupplier =
        () -> {
          Pose2d currentPose = drive.getPose();
          Translation2d hubCenter = FieldConstants.Hub.topCenterPoint.toTranslation2d();
          Translation2d hubToRobot = currentPose.getTranslation().minus(hubCenter);

          Translation2d targetTranslation =
              hubToRobot.getNorm() > 1e-6
                  ? hubCenter.plus(hubToRobot.times(2.5 / hubToRobot.getNorm()))
                  : hubCenter.plus(new Translation2d(2.5, 0.0));

          Pose2d aimedPose = ShotCalculator.getStationaryAimedPose(targetTranslation);
          Pose2d bluePose =
              aimedPose.getX() <= 3.0
                  ? aimedPose
                  : ShotCalculator.getStationaryAimedPose(
                      new Translation2d(3.0, targetTranslation.getY()));

          return AllianceFlipUtil.apply(bluePose);
        };

    Pose2d targetPose = targetPoseSupplier.get();

    Command pathfindCommand =
        Commands.defer(
            () ->
                AutoBuilder.pathfindToPose(
                    targetPose, new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), 0.0),
            Set.<Subsystem>of(drive));

    Command shootSequence =
        Commands.sequence(
            choreographer.setGoalCommand(Choreographer.Goal.SCORE_HUB),
            Commands.waitUntil(choreographer::isReadyToShoot),
            Commands.waitUntil(choreographer::isDoneShooting).withTimeout(10.0),
            choreographer.setGoalCommand(Choreographer.Goal.IDLE));

    // Run pathfinding and shooting once when auto starts.
    routine
        .active()
        .onTrue(
            Commands.sequence(
                pathfindCommand.withTimeout(10.0),
                new DriveToPose(drive, () -> targetPose),
                shootSequence));

    // Keep launch-mode heading assist active when the pathfinder is not using the drive subsystem.
    routine.active().whileTrue(robotContainer.driveCommand.launchModeCommand());

    return routine;
  }

  public AutoRoutine driveBackPreloadAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Drive Back Preload Auto");

    Command shootSequence =
        Commands.sequence(
            choreographer.setGoalCommand(Choreographer.Goal.SCORE_HUB),
            Commands.waitUntil(choreographer::isReadyToShoot),
            Commands.deadline(
                Commands.waitUntil(choreographer::isDoneShooting).withTimeout(10.0),
                Commands.repeatingSequence(
                    intake.deployCommand(),
                    Commands.waitSeconds(1),
                    intake.stowCommand(),
                    Commands.waitSeconds(1))),
            choreographer.setGoalCommand(Choreographer.Goal.IDLE));

    Command pathfindCommand =
        Commands.sequence(
            Commands.defer(
                () ->
                    AutoBuilder.pathfindToPose(
                        AllianceFlipUtil.apply(
                            new Pose2d(2, FieldConstants.fieldWidth / 2.0, Rotation2d.kZero)),
                        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
                        0.0),
                Set.<Subsystem>of(drive)),
            Commands.run(drive::stopWithX).withTimeout(.1));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                pathfindCommand, intake.homeCommand().withTimeout(0.5), shootSequence));

    return routine;
  }

  public AutoRoutine depotAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Depot Auto");

    // Load the two split segments from SimpleShoot.traj
    AutoTrajectory toDepot = routine.trajectory("DepotAuto", 0); // start → depot
    AutoTrajectory toScore = routine.trajectory("DepotAuto", 1); // depot → score
    // When the routine starts, drive from the robot's current estimated pose.
    routine.active().onTrue(toDepot.cmd());

    // Hold the depot end pose while waiting for the human player to load, then drive to score.
    // DriveToPose runs until toScore.cmd() interrupts it (drive is a shared requirement).
    toDepot
        .done()
        .onTrue(Commands.sequence(holdFinalPose(toDepot).withTimeout(5.0), toScore.cmd()));

    // Once the score trajectory finishes, hold the score pose while the shooting sequence runs.
    // DriveToPose is cancelled automatically when the routine ends or drive is re-required.
    toScore
        .done()
        .onTrue(
            Commands.parallel(scoreAtHubCommand(), delayedRetractIntakeAfterShotStartCommand()));

    return routine;
  }

  public AutoRoutine groundDepotAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Ground Depot Auto");

    AutoTrajectory toDepot = routine.trajectory("DepotAutoGround", 0);
    AutoTrajectory creepAtDepot = routine.trajectory("DepotAutoGround", 1);
    AutoTrajectory toScore = routine.trajectory("DepotAutoGround", 2);

    routine.active().onTrue(toDepot.cmd());

    creepAtDepot.active().whileTrue(intake.runCommand());

    toDepot.done().onTrue(intake.deployCommand());
    toDepot.done().onTrue(creepAtDepot.cmd());

    creepAtDepot.done().onTrue(toScore.cmd());

    toScore
        .done()
        .onTrue(
            Commands.parallel(scoreAtHubCommand(), delayedRetractIntakeAfterShotStartCommand()));

    return routine;
  }

  public AutoRoutine groundAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Ground Auto");

    AutoTrajectory toGround = routine.trajectory("GrabFromGround", 0);
    AutoTrajectory creepAlongGround = routine.trajectory("GrabFromGround", 1);
    AutoTrajectory toScore = routine.trajectory("GrabFromGround", 2);

    routine.active().onTrue(toGround.cmd());

    creepAlongGround.active().whileTrue(intake.runCommand());

    toGround.done().onTrue(intake.deployCommand());
    toGround.done().onTrue(creepAlongGround.cmd());

    creepAlongGround.done().onTrue(toScore.cmd());

    toScore
        .done()
        .onTrue(
            Commands.parallel(scoreAtHubCommand(), delayedRetractIntakeAfterShotStartCommand()));

    return routine;
  }

  public AutoRoutine rushToCenterAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Rush To Center Auto");

    AutoTrajectory rushToCenter = routine.trajectory("RushToCenter", 0);
    AutoTrajectory creepUpCenter = routine.trajectory("RushToCenter", 1);
    AutoTrajectory rushAcrossCenter = routine.trajectory("RushToCenter", 2);
    AutoTrajectory creepBackToScore = routine.trajectory("RushToCenter", 3);

    routine.active().onTrue(rushToCenter.cmd());

    creepUpCenter.active().whileTrue(intake.runCommand());
    creepBackToScore.active().whileTrue(intake.runCommand());

    rushToCenter.done().onTrue(intake.deployCommand());
    rushToCenter.done().onTrue(creepUpCenter.cmd());

    creepUpCenter.done().onTrue(rushAcrossCenter.cmd());

    rushAcrossCenter.done().onTrue(creepBackToScore.cmd());

    creepBackToScore
        .done()
        .onTrue(
            Commands.parallel(scoreAtHubCommand(), delayedRetractIntakeAfterShotStartCommand()));

    return routine;
  }

  private Command scoreAtHubCommand() {
    Command scoreSequence =
        Commands.sequence(
            choreographer.setGoalCommand(Choreographer.Goal.SCORE_HUB),
            Commands.waitUntil(choreographer::isReadyToShoot),
            Commands.waitUntil(choreographer::isDoneShooting).withTimeout(10.0),
            choreographer.setGoalCommand(Choreographer.Goal.IDLE));

    return Commands.deadline(scoreSequence, robotContainer.driveCommand.launchModeCommand());
  }

  private Command delayedRetractIntakeAfterShotStartCommand() {
    return Commands.sequence(
        Commands.waitUntil(choreographer::isReadyToShoot),
        Commands.waitSeconds(1.0),
        intake.stowCommand(),
        Commands.runOnce(intake::run),
        Commands.waitSeconds(0.2),
        intake.stopCommand());
  }

  private Optional<Pose2d> startPoseOf(
      Supplier<AutoRoutine> routineSupplier,
      Function<AutoRoutine, AutoTrajectory> trajectorySupplier) {
    return trajectorySupplier.apply(routineSupplier.get()).getInitialPose();
  }

  /**
   * Returns a {@link DriveToPose} command that holds the final pose of the given trajectory. If the
   * trajectory has no final pose (empty), returns {@link Commands#none()}.
   */
  private DriveToPose holdFinalPose(AutoTrajectory trajectory) {
    var finalPose = trajectory.getFinalPose();
    return new DriveToPose(drive, () -> finalPose.orElseThrow());
  }
}
