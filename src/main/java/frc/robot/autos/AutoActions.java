// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Choreographer;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

public final class AutoActions {
  private final Choreographer choreographer;
  private final Intake intake;
  private final RobotContainer robotContainer;

  public AutoActions(Choreographer choreographer, Intake intake, RobotContainer robotContainer) {
    this.choreographer = choreographer;
    this.intake = intake;
    this.robotContainer = robotContainer;
  }

  private Command baseScore() {
    return Commands.sequence(
            Commands.runOnce(() -> choreographer.setGoal(Choreographer.Goal.REVERSE_INDEXER)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> choreographer.setGoal(Choreographer.Goal.SCORE_HUB)),
            Commands.waitUntil(choreographer::isReadyToShoot),
            Commands.idle()) // hold SCORE_HUB; withTimeout in AutoRoutines is the exit
        .deadlineFor(robotContainer.driveCommand.launchModeAndStopCommand())
        .finallyDo(
            interrupted -> {
              Logger.recordOutput("AutoActions/BaseScoreFinished", true);
              choreographer.setGoal(Choreographer.Goal.IDLE);
            })
        .beforeStarting(() -> Logger.recordOutput("AutoActions/BaseScoreStarted", true));
  }

  public Command scoreAtHub() {
    return baseScore();
  }

  public Command scoreWithAgitation() {
    return Commands.parallel(baseScore(), intakePulse());
  }

  public Command scoreWithReverseAgitationAndRollers() {
    return Commands.parallel(baseScore(), intakePulseReverseWithRollers());
  }

  public Command intakePulse() {
    return Commands.repeatingSequence(
        intake.deployCommand(),
        Commands.waitSeconds(1.0),
        intake.stowCommand(),
        Commands.waitSeconds(1.0));
  }

  /** Agitate in reverse (stow→deploy) and run rollers throughout, to clear stuck balls. */
  public Command intakePulseReverseWithRollers() {
    return Commands.repeatingSequence(
        Commands.runOnce(() -> { intake.stow(); intake.run(); }, intake),
        Commands.waitSeconds(1.0),
        Commands.runOnce(() -> { intake.deploy(); intake.run(); }, intake),
        Commands.waitSeconds(1.0));
  }

  public Command stowAndHome() {
    return Commands.sequence(intake.stowCommand(), intake.homeCommand());
  }
}
