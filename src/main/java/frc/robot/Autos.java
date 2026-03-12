// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466
package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.Choreographer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;

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
 * Represents a single trajectory (or split segment) loaded from a Choreo project file. Exposes
 * triggers for reacting to trajectory lifecycle events:
 *
 * <ul>
 *   <li>{@code active()} / {@code inactive()} — true while the trajectory is / isn't running
 *   <li>{@code done()} — pulses true for <b>one cycle</b> when the trajectory finishes
 *   <li>{@code atTime(double)} / {@code atTime(String)} — pulses at a timestamp or named event
 *       marker
 *   <li>{@code atTimeBeforeEnd(double)} — pulses N seconds before the trajectory ends
 *   <li>{@code atPose(String, m, rad)} — true when the robot is near an event marker's pose
 *   <li>{@code chain(other)} — shorthand for {@code done().onTrue(other.cmd())}
 * </ul>
 *
 * <h2>Trajectory Organization in Choreo</h2>
 *
 * There are two strategies for organizing trajectory segments. Choose based on whether you need
 * <b>branching</b> (runtime decisions about which path to follow next).
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
  private final RobotContainer robotContainer;
  private final Choreographer choreographer;
  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;

  public Autos(RobotContainer robotContainer, Choreographer choreographer) {
    this.robotContainer = robotContainer;
    this.choreographer = choreographer;

    autoFactory = new AutoFactory(robotContainer.drive::getPose, robotContainer.drive::setPose, robotContainer.drive::followTrajectory, true, robotContainer.drive);

    autoChooser = new AutoChooser();

    // Register all autonomous routines here.
    // Use method references so routines are lazily constructed only when selected.
    autoChooser.addRoutine("Depot Auto", this::depotAuto);

    autoChooser.addCmd("Do Nothing", () - >Commands.none());
  }

  /**
   * Returns the {@link AutoChooser} to be placed on the dashboard for driver selection.
   *
   * @return the auto chooser widget
   */
  public AutoChooser getAutoChooser() {
    return autoChooser;
  }

  public AutoRoutine shootFromAnywhereAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Preload Auto");

    // No Trajectory
    // No odometry reset, just call the choreographer shoot command
    routine.active().whileTrue(
    Commands.parallel(robotContainer.driveCommand.launchModeCommand(), Commands.sequence(
    choreographer.setGoalCommand(Choreographer.Goal.SCORE_HUB), Commands.waitUntil(choreographer::isReadyToShoot), Commands.waitUntil(choreographer::isDoneShooting).withTimeout(10.0), choreographer.setGoalCommand(Choreographer.Goal.IDLE))));

    return routine;
  }

  public AutoRoutine depotAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Depot Auto");

    // Load the two split segments from SimpleShoot.traj
    AutoTrajectory toDepot = routine.trajectory("SimpleShoot", 0); // start → depot
    AutoTrajectory toScore = routine.trajectory("SimpleShoot", 1); // depot → score
    // When the routine starts, reset odometry and drive to the depot
    routine.active().onTrue(Commands.sequence(toDepot.resetOdometry(), toDepot.cmd()));

    // Hold the depot end pose while waiting for the human player to load, then drive to score.
    // DriveToPose runs until toScore.cmd() interrupts it (drive is a shared requirement).
    toDepot.done().onTrue(Commands.sequence(holdFinalPose(toDepot).withTimeout(5.0), toScore.cmd()));

    // Once the score trajectory finishes, hold the score pose while the shooting sequence runs.
    // DriveToPose is cancelled automatically when the routine ends or drive is re-required.
    toScore.done().onTrue(
    Commands.parallel(
    holdFinalPose(toScore), Commands.sequence(
    choreographer.setGoalCommand(Choreographer.Goal.SCORE_HUB), Commands.waitUntil(choreographer::isReadyToShoot), Commands.waitUntil(choreographer::isDoneShooting).withTimeout(10.0), choreographer.setGoalCommand(Choreographer.Goal.IDLE)));

    return routine;
  }

  /**
   * Returns a {@link DriveToPose} command that holds the final pose of the given trajectory. If the
   * trajectory has no final pose (empty), returns {@link Commands#none()}.
   */
  private DriveToPose holdFinalPose(AutoTrajectory trajectory) {
    var finalPose = trajectory.getFinalPose();
    return new DriveToPose(drive, () - >finalPose.orElseThrow());
  }
}