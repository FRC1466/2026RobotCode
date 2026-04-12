// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Choreographer;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

/** Reusable game-action commands composed into auto routines. */
final class AutoActions {

  private final Choreographer choreographer;
  private final Intake intake;
  private final RobotContainer rc;

  AutoActions(RobotContainer rc) {
    this.choreographer = rc.getChoreographer();
    this.intake = rc.getIntake();
    this.rc = rc;
  }

  /**
   * Core scoring sequence. Primes the indexer, commands SCORE_HUB, waits until the shooter is
   * ready, then holds until cancelled (caller supplies a timeout or deadline).
   *
   * <p>The drive holds its launch-mode heading during the sequence and stops when it ends.
   * Choreographer is returned to IDLE on completion or interruption.
   */
  private Command baseScore() {
    return Commands.sequence(
            Commands.runOnce(() -> choreographer.setGoal(Choreographer.Goal.REVERSE_INDEXER)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> choreographer.setGoal(Choreographer.Goal.SCORE_HUB)),
            Commands.waitUntil(choreographer::isReadyToShoot),
            Commands.idle())
        .deadlineFor(rc.driveCommand.launchModeAndStopCommand())
        .finallyDo(
            interrupted -> {
              Logger.recordOutput("Auto/BaseScoreFinished", true);
              choreographer.setGoal(Choreographer.Goal.IDLE);
            })
        .beforeStarting(() -> Logger.recordOutput("Auto/BaseScoreStarted", true));
  }

  Command scoreAtHub() {
    return baseScore();
  }

  Command scoreWithAgitation() {
    return Commands.parallel(baseScore(), intakePulse());
  }

  /** Score while agitating the intake in reverse with rollers running to clear stuck balls. */
  Command scoreWithReverseAgitationAndRollers() {
    return Commands.parallel(baseScore(), intakePulseReverseWithRollers());
  }

  /** Repeatedly deploy and stow the intake to agitate balls forward. */
  Command intakePulse() {
    return Commands.repeatingSequence(
        intake.deployCommand(),
        Commands.waitSeconds(1.0),
        intake.stowCommand(),
        Commands.waitSeconds(1.0));
  }

  /** Agitate in reverse (stow → deploy) with rollers running throughout, to clear stuck balls. */
  Command intakePulseReverseWithRollers() {
    return Commands.repeatingSequence(
        Commands.runOnce(
            () -> {
              intake.stow();
              intake.run();
            },
            intake),
        Commands.waitSeconds(1.0),
        Commands.runOnce(
            () -> {
              intake.deploy();
              intake.run();
            },
            intake),
        Commands.waitSeconds(1.0));
  }

  Command stowAndHome() {
    return Commands.sequence(intake.stowCommand(), intake.homeCommand());
  }
}
