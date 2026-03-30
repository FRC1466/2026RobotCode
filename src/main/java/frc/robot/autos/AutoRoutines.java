// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Choreographer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

@SuppressWarnings("unused")
public final class AutoRoutines {
  private final RobotContainer robotContainer;
  private final Drive drive;
  private final Choreographer choreographer;
  private final Intake intake;
  private final AutoFactory autoFactory;
  private final AutoActions actions;

  public AutoRoutines(RobotContainer robotContainer, AutoFactory autoFactory, AutoActions actions) {
    this.robotContainer = robotContainer;
    this.drive = robotContainer.getDrive();
    this.choreographer = robotContainer.getChoreographer();
    this.intake = robotContainer.getIntake();
    this.autoFactory = autoFactory;
    this.actions = actions;
  }

  public AutoRoutine driveBackPreloadAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Drive Back Preload Auto");
    AutoTrajectory trajectory = routine.trajectory("CentralAuto");

    routine.active().onTrue(trajectory.cmd());
    trajectory
        .done()
        .onTrue(
            Commands.sequence(actions.scoreWithAgitation().withTimeout(3), actions.stowAndHome()));

    return routine;
  }

  public AutoRoutine driveLeftPreloadAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Drive Left Preload Auto");
    AutoTrajectory trajectory = routine.trajectory("LeftAuto");

    routine.active().onTrue(trajectory.cmd());
    trajectory
        .done()
        .onTrue(
            Commands.sequence(actions.scoreWithAgitation().withTimeout(3), actions.stowAndHome()));

    return routine;
  }

  public AutoRoutine LeftPreloadAuto() {
    return driveLeftPreloadAuto();
  }

  public AutoRoutine outpostAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Outpost Auto");

    AutoTrajectory[] t = trajectories(routine, "OutpostAuto", 3);
    AutoTrajectory toOutpost = t[0];
    AutoTrajectory creepAtOutpost = t[1];
    AutoTrajectory toScore = t[2];

    routine.active().onTrue(toOutpost.cmd());

    creepAtOutpost.active().whileTrue(intake.runCommand());

    toOutpost.done().onTrue(intake.deployCommand());
    toOutpost.done().onTrue(creepAtOutpost.cmd());

    creepAtOutpost.done().onTrue(toScore.cmd());
    toScore.done().onTrue(actions.scoreWithAgitation().withTimeout(3));

    return routine;
  }

  public AutoRoutine preloadThenOutpostAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Preload Then Outpost Auto");

    AutoTrajectory toScore = routine.trajectory("OutpostGroundPreload", 0);
    AutoTrajectory transition = routine.trajectory("OutpostGroundPreload", 1);
    AutoTrajectory creepAtOutpost = routine.trajectory("OutpostGroundPreload", 2);
    AutoTrajectory toScoreAgain = routine.trajectory("OutpostGroundPreload", 3);

    routine.active().onTrue(toScore.cmd());

    toScore
        .done()
        .onTrue(
            actions
                .scoreWithAgitation()
                .withTimeout(3)
                .andThen(Commands.runOnce(transition.cmd()::schedule)));

    transition.active().onTrue(intake.deployCommand());
    transition.done().onTrue(creepAtOutpost.cmd());
    creepAtOutpost.active().whileTrue(intake.runAtTargetSpeedCommand());
    creepAtOutpost.done().onTrue(toScoreAgain.cmd());

    toScoreAgain.done().onTrue(actions.scoreWithAgitation().withTimeout(3));

    return routine;
  }

  public AutoRoutine groundAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Ground Auto");

    AutoTrajectory[] t = trajectories(routine, "GrabFromGround", 3);
    AutoTrajectory toGround = t[0];
    AutoTrajectory creepAlongGround = t[1];
    AutoTrajectory toScore = t[2];

    routine.active().onTrue(toGround.cmd());

    creepAlongGround.active().whileTrue(intake.runCommand());

    toGround.done().onTrue(intake.deployCommand());
    toGround.done().onTrue(creepAlongGround.cmd());

    creepAlongGround.done().onTrue(toScore.cmd());
    toScore.done().onTrue(actions.scoreWithAgitation().withTimeout(3));

    return routine;
  }

  public AutoRoutine preloadThenGroundAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Preload Then Ground Auto");

    AutoTrajectory[] t = trajectories(routine, "GrabFromGround", 3);
    AutoTrajectory toScore = t[0];
    AutoTrajectory creepAlongGround = t[1];
    AutoTrajectory toScoreAgain = t[2];

    routine.active().onTrue(toScore.cmd());

    creepAlongGround.active().whileTrue(intake.runCommand());

    toScore
        .done()
        .onTrue(
            actions
                .scoreWithAgitation()
                .withTimeout(3)
                .andThen(intake.deployCommand())
                .andThen(Commands.runOnce(creepAlongGround.cmd()::schedule)));

    creepAlongGround.done().onTrue(toScoreAgain.cmd());
    toScoreAgain.done().onTrue(actions.scoreWithAgitation().withTimeout(3));

    return routine;
  }

  public AutoRoutine doubleGroundPickupAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Double Ground Pickup Auto");

    AutoTrajectory[] t1 = trajectories(routine, "RushToCenterUnoDip", 3);
    AutoTrajectory toGround1 = t1[0];
    AutoTrajectory creepAlongGround1 = t1[1];
    AutoTrajectory toScore1 = t1[2];

    AutoTrajectory[] t2 = trajectories(routine, "RushToCenterDosDip", 3);
    AutoTrajectory toGround2 = t2[0];
    AutoTrajectory creepAlongGround2 = t2[1];
    AutoTrajectory toScore2 = t2[2];

    routine.active().onTrue(toGround1.cmd());

    creepAlongGround1.active().whileTrue(intake.runCommand());
    creepAlongGround2.active().whileTrue(intake.runCommand());

    toGround1.done().onTrue(intake.deployCommand());
    toGround1.done().onTrue(creepAlongGround1.cmd());

    creepAlongGround1.done().onTrue(toScore1.cmd());
    toScore1
        .done()
        .onTrue(
            actions
                .scoreWithAgitation()
                .withTimeout(3)
                .andThen(Commands.runOnce(toGround2.cmd()::schedule)));

    toGround2.done().onTrue(intake.deployCommand());
    toGround2.done().onTrue(creepAlongGround2.cmd());

    creepAlongGround2.done().onTrue(toScore2.cmd());
    toScore2.done().onTrue(actions.scoreWithAgitation().withTimeout(3));

    return routine;
  }

  public AutoRoutine singleGroundPickupAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Single Ground Pickup Auto");

    AutoTrajectory[] t = trajectories(routine, "RushToCenterUnoDip", 3);
    AutoTrajectory toNeutralZone = t[0];
    AutoTrajectory creepInNeutralZone = t[1];
    AutoTrajectory toScore = t[2];

    routine.active().onTrue(toNeutralZone.cmd());

    creepInNeutralZone.active().whileTrue(intake.runCommand());

    toNeutralZone.done().onTrue(intake.deployCommand());
    toNeutralZone.done().onTrue(creepInNeutralZone.cmd());

    creepInNeutralZone.done().onTrue(toScore.cmd());
    toScore.done().onTrue(actions.scoreWithAgitation().withTimeout(3));

    return routine;
  }

  public AutoRoutine bumpy() {
    AutoRoutine routine = autoFactory.newRoutine("Bumpy");
    AutoTrajectory trajectory = routine.trajectory("Bumpy");

    routine.active().onTrue(trajectory.cmd());
    return routine;
  }

  public Optional<Pose2d> startPoseOf(
      Supplier<AutoRoutine> routineSupplier,
      Function<AutoRoutine, AutoTrajectory> trajectorySupplier) {
    return trajectorySupplier.apply(routineSupplier.get()).getInitialPose();
  }

  private AutoTrajectory[] trajectories(AutoRoutine routine, String name, int count) {
    AutoTrajectory[] trajectories = new AutoTrajectory[count];
    for (int i = 0; i < count; i++) {
      trajectories[i] = routine.trajectory(name, i);
    }
    return trajectories;
  }
}
