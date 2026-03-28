// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Choreographer;
import frc.robot.subsystems.intake.Intake;

public final class AutoActions {
  private static final double SHOOT_DONE_TIMEOUT_SECONDS = 10.0;

  private final Choreographer choreographer;
  private final Intake intake;

  public AutoActions(Choreographer choreographer, Intake intake) {
    this.choreographer = choreographer;
    this.intake = intake;
  }

  public Command scoreAtHub() {
    return Commands.sequence(
        choreographer.setGoalCommand(Choreographer.Goal.SCORE_HUB),
        Commands.waitUntil(choreographer::isReadyToShoot),
        Commands.waitUntil(choreographer::isDoneShooting).withTimeout(SHOOT_DONE_TIMEOUT_SECONDS),
        choreographer.setGoalCommand(Choreographer.Goal.IDLE));
  }

  public Command scoreAtHub(Command driveAssist) {
    return Commands.deadline(scoreAtHub(), driveAssist);
  }

  public Command scoreWithAgitation() {
    return Commands.parallel(scoreAtHub(), intakePulse());
  }

  public Command scoreWithAgitation(Command driveAssist) {
    return Commands.parallel(scoreAtHub(driveAssist), intakePulse());
  }

  public Command intakePulse() {
    return Commands.repeatingSequence(
        intake.deployCommand(),
        Commands.waitSeconds(1.0),
        intake.stowCommand(),
        Commands.waitSeconds(1.0));
  }

  public Command stowAndHome() {
    return Commands.sequence(intake.stowCommand(), intake.homeCommand());
  }
}
