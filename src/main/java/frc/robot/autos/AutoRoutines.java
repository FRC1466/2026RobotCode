// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.lib.BLine.Path;
import org.littletonrobotics.junction.Logger;

/**
 * Factory methods for autonomous routines. Each method returns a self-contained {@link Command}.
 *
 * <p>Two categories:
 *
 * <ul>
 *   <li><b>Code-only</b> — paths are defined inline with {@link Path.Waypoint} / {@link
 *       Path.TranslationTarget} etc. Ready to run immediately.
 *   <li><b>GUI-path stubs</b> — paths must be authored in BLine-GUI and exported to {@code
 *       deploy/autos/paths/}. Each stub prints a clear warning and does nothing until the JSON
 *       files exist. Replace the stub body with the documented implementation once the files are
 *       ready.
 * </ul>
 */
final class AutoRoutines {

  private final AutoManager manager;
  private final AutoActions actions;
  private final RobotContainer rc;

  AutoRoutines(AutoManager manager, AutoActions actions, RobotContainer rc) {
    this.manager = manager;
    this.actions = actions;
    this.rc = rc;
  }

  // ── Code-only routines ────────────────────────────────────────────────────

  /**
   * Drive straight back from the center start line, then score the preloaded ball.
   *
   * <p>Path coordinates sourced from {@code CentralAuto.traj} waypoints: start (3.6, 4.02, 0°) →
   * end (1.75, 4.02, 0°).
   */
  Command driveBackPreload() {
    Path path =
        new Path(
            new Path.Waypoint(new Pose2d(3.6, 4.02, Rotation2d.kZero)),
            new Path.Waypoint(new Pose2d(1.75, 4.02, Rotation2d.kZero)));

    return Commands.sequence(
            manager.follow(path),
            actions.scoreWithReverseAgitationAndRollers().withTimeout(3),
            actions.stowAndHome())
        .withName("Drive Back Preload Auto");
  }

  /**
   * Drive from the left start position back toward the alliance station, then score.
   *
   * <p>Path coordinates sourced from {@code LeftAuto.traj} waypoints: start (3.38, 6.53, −63.6°) →
   * end (2.25, 5.52, −31.8°).
   */
  Command driveLeftPreload() {
    Path path =
        new Path(
            new Path.Waypoint(new Pose2d(3.38, 6.53, Rotation2d.fromRadians(-1.1104))),
            new Path.Waypoint(new Pose2d(2.25, 5.52, Rotation2d.fromRadians(-0.5542))));

    return Commands.sequence(
            manager.follow(path),
            actions.scoreWithReverseAgitationAndRollers().withTimeout(3),
            actions.stowAndHome())
        .withName("Drive Left Preload Auto");
  }

  // ── GUI-path stubs ────────────────────────────────────────────────────────
  //
  // Each stub below documents:
  //   1. The JSON path files to create in BLine-GUI
  //   2. The intended routine logic to implement once those files exist
  //
  // Export paths from BLine-GUI to: deploy/autos/paths/<name>.json
  // Then replace the guiPathStub(...) call with the documented implementation.

  /**
   * TODO: Author in BLine-GUI, then implement.
   *
   * <p>Path files needed in {@code deploy/autos/paths/}:
   *
   * <ul>
   *   <li>{@code outpost_rush.json} — fast run from start to outpost entrance
   *   <li>{@code outpost_creep.json} — slow creep through outpost pickup zone (intake runs)
   *   <li>{@code outpost_return.json} — return to scoring position
   * </ul>
   *
   * <p>Implementation once paths exist:
   *
   * <pre>{@code
   * return Commands.sequence(
   *     manager.followWithReset(new Path("outpost_rush")),
   *     Commands.parallel(
   *         rc.getIntake().deployCommand(),
   *         Commands.sequence(
   *             manager.follow(new Path("outpost_creep")).deadlineFor(rc.getIntake().runCommand()),
   *             manager.follow(new Path("outpost_return")))),
   *     actions.scoreWithReverseAgitationAndRollers().withTimeout(5))
   *     .withName("Outpost Auto");
   * }</pre>
   */
  Command outpostAuto() {
    return guiPathStub(
        "Outpost Auto", "outpost_rush.json", "outpost_creep.json", "outpost_return.json");
  }

  /**
   * TODO: Author in BLine-GUI, then implement.
   *
   * <p>Path files needed in {@code deploy/autos/paths/}:
   *
   * <ul>
   *   <li>{@code preload_outpost_to_score.json} — drive to scoring position for preload
   *   <li>{@code preload_outpost_transition.json} — move toward outpost after scoring
   *   <li>{@code preload_outpost_creep.json} — slow creep through outpost pickup zone
   *   <li>{@code preload_outpost_return.json} — return to scoring position
   * </ul>
   *
   * <p>Implementation once paths exist:
   *
   * <pre>{@code
   * return Commands.sequence(
   *     manager.followWithReset(new Path("preload_outpost_to_score")),
   *     actions.scoreWithReverseAgitationAndRollers().withTimeout(3),
   *     Commands.parallel(
   *         rc.getIntake().deployCommand(),
   *         manager.follow(new Path("preload_outpost_transition"))),
   *     manager.follow(new Path("preload_outpost_creep")).deadlineFor(rc.getIntake().runAtTargetSpeedCommand()),
   *     manager.follow(new Path("preload_outpost_return")),
   *     actions.scoreWithReverseAgitationAndRollers().withTimeout(10))
   *     .withName("Preload Then Outpost Auto");
   * }</pre>
   */
  Command preloadThenOutpostAuto() {
    return guiPathStub(
        "Preload Then Outpost Auto",
        "preload_outpost_to_score.json",
        "preload_outpost_transition.json",
        "preload_outpost_creep.json",
        "preload_outpost_return.json");
  }

  /**
   * TODO: Author in BLine-GUI, then implement.
   *
   * <p>Path files needed in {@code deploy/autos/paths/}:
   *
   * <ul>
   *   <li>{@code ground_rush.json} — fast rush to ground ball pickup zone
   *   <li>{@code ground_creep.json} — slow creep through ground pickup zone (intake runs)
   *   <li>{@code ground_return.json} — return to scoring position
   * </ul>
   *
   * <p>Implementation once paths exist:
   *
   * <pre>{@code
   * return Commands.sequence(
   *     manager.followWithReset(new Path("ground_rush")),
   *     Commands.parallel(
   *         rc.getIntake().deployCommand(),
   *         Commands.sequence(
   *             manager.follow(new Path("ground_creep")).deadlineFor(rc.getIntake().runCommand()),
   *             manager.follow(new Path("ground_return")))),
   *     actions.scoreWithReverseAgitationAndRollers().withTimeout(3))
   *     .withName("Ground Auto");
   * }</pre>
   */
  Command groundAuto() {
    return guiPathStub(
        "Ground Auto", "ground_rush.json", "ground_creep.json", "ground_return.json");
  }

  /**
   * TODO: Author in BLine-GUI, then implement.
   *
   * <p>Path files needed in {@code deploy/autos/paths/}:
   *
   * <ul>
   *   <li>{@code preload_ground_to_score.json} — drive to scoring position for preload
   *   <li>{@code preload_ground_creep.json} — slow creep through ground pickup zone (intake runs)
   *   <li>{@code preload_ground_return.json} — return to scoring position after pickup
   * </ul>
   *
   * <p>Implementation once paths exist:
   *
   * <pre>{@code
   * return Commands.sequence(
   *     manager.followWithReset(new Path("preload_ground_to_score")),
   *     actions.scoreWithReverseAgitationAndRollers().withTimeout(3),
   *     Commands.parallel(
   *         rc.getIntake().deployCommand(),
   *         manager.follow(new Path("preload_ground_creep")).deadlineFor(rc.getIntake().runCommand())),
   *     manager.follow(new Path("preload_ground_return")),
   *     actions.scoreWithReverseAgitationAndRollers().withTimeout(3))
   *     .withName("Preload Then Ground Auto");
   * }</pre>
   */
  Command preloadThenGroundAuto() {
    return guiPathStub(
        "Preload Then Ground Auto",
        "preload_ground_to_score.json",
        "preload_ground_creep.json",
        "preload_ground_return.json");
  }

  /**
   * TODO: Author in BLine-GUI, then implement.
   *
   * <p>Path files needed in {@code deploy/autos/paths/}:
   *
   * <ul>
   *   <li>{@code double_rush1.json} — first rush to neutral zone
   *   <li>{@code double_creep1.json} — first slow creep through pickup zone (ball 1)
   *   <li>{@code double_return1.json} — return to score after ball 1
   *   <li>{@code double_rush2.json} — second rush to neutral zone
   *   <li>{@code double_creep2.json} — second slow creep through pickup zone (ball 2)
   *   <li>{@code double_return2.json} — return to score after ball 2
   * </ul>
   *
   * <p>Intake timing note: deploy intake during each rush path so it is fully extended before the
   * creep sweep begins. This matches the original behavior where intake deployment happened in
   * parallel with the orient pivot, giving the full sweep to complete.
   *
   * <p>Implementation once paths exist:
   *
   * <pre>{@code
   * return Commands.sequence(
   *     // Cycle 1
   *     Commands.parallel(
   *         rc.getIntake().deployCommand(),
   *         manager.followWithReset(new Path("double_rush1"))),
   *     manager.follow(new Path("double_creep1")).deadlineFor(rc.getIntake().runCommand()),
   *     manager.follow(new Path("double_return1")),
   *     actions.scoreWithReverseAgitationAndRollers().withTimeout(5),
   *     // Cycle 2
   *     Commands.parallel(
   *         rc.getIntake().deployCommand(),
   *         manager.follow(new Path("double_rush2"))),
   *     manager.follow(new Path("double_creep2")).deadlineFor(rc.getIntake().runCommand()),
   *     manager.follow(new Path("double_return2")),
   *     actions.scoreWithReverseAgitationAndRollers().withTimeout(10))
   *     .withName("Double Ground Pickup Auto");
   * }</pre>
   */
  Command doubleGroundPickupAuto() {
    return guiPathStub(
        "Double Ground Pickup Auto",
        "double_rush1.json",
        "double_creep1.json",
        "double_return1.json",
        "double_rush2.json",
        "double_creep2.json",
        "double_return2.json");
  }

  /**
   * TODO: Author in BLine-GUI, then implement.
   *
   * <p>Path files needed in {@code deploy/autos/paths/}:
   *
   * <ul>
   *   <li>{@code single_rush.json} — rush to neutral zone
   *   <li>{@code single_creep.json} — slow creep through pickup zone (intake runs)
   *   <li>{@code single_return.json} — return to scoring position
   * </ul>
   *
   * <p>Sequence: score preload → rush → creep (intake running) → return → score.
   *
   * <p>Implementation once paths exist:
   *
   * <pre>{@code
   * return Commands.sequence(
   *     actions.scoreWithReverseAgitationAndRollers().withTimeout(3),
   *     Commands.parallel(
   *         rc.getIntake().deployCommand(),
   *         manager.followWithReset(new Path("single_rush"))),
   *     manager.follow(new Path("single_creep")).deadlineFor(rc.getIntake().runCommand()),
   *     manager.follow(new Path("single_return")),
   *     actions.scoreWithReverseAgitationAndRollers().withTimeout(10))
   *     .withName("Single Ground Pickup Auto");
   * }</pre>
   */
  Command singleGroundPickupAuto() {
    return guiPathStub(
        "Single Ground Pickup Auto", "single_rush.json", "single_creep.json", "single_return.json");
  }

  /**
   * TODO: Author in BLine-GUI, then implement.
   *
   * <p>Path files needed in {@code deploy/autos/paths/}:
   *
   * <ul>
   *   <li>{@code one_dip_left_rush.json} — rush to left-side neutral zone
   *   <li>{@code one_dip_left_creep.json} — slow creep through left pickup zone (intake runs)
   *   <li>{@code one_dip_left_return.json} — return to scoring position
   * </ul>
   *
   * <p>Sequence: score preload → rush → creep → return → score. Mirror of {@link
   * #singleGroundPickupAuto} on the left side.
   *
   * <p>Implementation once paths exist:
   *
   * <pre>{@code
   * return Commands.sequence(
   *     actions.scoreWithReverseAgitationAndRollers().withTimeout(3),
   *     Commands.parallel(
   *         rc.getIntake().deployCommand(),
   *         manager.followWithReset(new Path("one_dip_left_rush"))),
   *     manager.follow(new Path("one_dip_left_creep")).deadlineFor(rc.getIntake().runCommand()),
   *     manager.follow(new Path("one_dip_left_return")),
   *     actions.scoreWithReverseAgitationAndRollers().withTimeout(10))
   *     .withName("One Dip Left Auto");
   * }</pre>
   */
  Command oneDipLeftAuto() {
    return guiPathStub(
        "One Dip Left Auto",
        "one_dip_left_rush.json",
        "one_dip_left_creep.json",
        "one_dip_left_return.json");
  }

  // ─────────────────────────────────────────────────────────────────────────

  /** Placeholder command that logs a clear warning and does nothing. */
  private Command guiPathStub(String routineName, String... pathFilesNeeded) {
    return Commands.runOnce(
            () -> {
              String msg =
                  "[AutoRoutines] "
                      + routineName
                      + " requires GUI-authored paths that have not been exported yet. "
                      + "Create the following files in BLine-GUI and export to deploy/autos/paths/: "
                      + String.join(", ", pathFilesNeeded);
              System.err.println(msg);
              Logger.recordOutput("Auto/StubWarning/" + routineName, msg);
            })
        .withName(routineName);
  }
}
