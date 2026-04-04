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
            Commands.sequence(actions.scoreWithReverseAgitationAndRollers().withTimeout(3), actions.stowAndHome()));

    return routine;
  }

  public AutoRoutine driveLeftPreloadAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Drive Left Preload Auto");
    AutoTrajectory trajectory = routine.trajectory("LeftAuto");

    routine.active().onTrue(trajectory.cmd());
    trajectory
        .done()
        .onTrue(
            Commands.sequence(actions.scoreWithReverseAgitationAndRollers().withTimeout(3), actions.stowAndHome()));

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
    toScore.done().onTrue(actions.scoreWithReverseAgitationAndRollers().withTimeout(5));

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
                .scoreWithReverseAgitationAndRollers()
                .withTimeout(3)
                .andThen(Commands.runOnce(transition.cmd()::schedule)));

    transition.active().onTrue(intake.deployCommand());
    transition.done().onTrue(creepAtOutpost.cmd());
    creepAtOutpost.active().whileTrue(intake.runAtTargetSpeedCommand());
    creepAtOutpost.done().onTrue(toScoreAgain.cmd());

    toScoreAgain.done().onTrue(actions.scoreWithReverseAgitationAndRollers().withTimeout(10));

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
    toScore.done().onTrue(actions.scoreWithReverseAgitationAndRollers().withTimeout(3));

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
                .scoreWithReverseAgitationAndRollers()
                .withTimeout(3)
                .andThen(intake.deployCommand())
                .andThen(Commands.runOnce(creepAlongGround.cmd()::schedule)));

    creepAlongGround.done().onTrue(toScoreAgain.cmd());
    toScoreAgain.done().onTrue(actions.scoreWithReverseAgitationAndRollers().withTimeout(3));

    return routine;
  }

  public AutoRoutine doubleGroundPickupAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Double Ground Pickup Auto");

    // UnoDip has 4 segments (splits at samples 0, 41, 52, 129):
    //   Seg 0 — fast rush along bottom to neutral zone entrance (~1.0 s, KeepInLane)
    //   Seg 1 — short orient pivot at neutral zone entrance (~0.3 s, PointAt ball cluster);
    //           deploy intake here so it's ready before the slow sweep begins
    //   Seg 2 — slow sweep through ball pickup zone (~1.9 s, MaxVelocity=1.0 m/s, PointAt);
    //           intake runs here — this is when ball #1 is collected
    //   Seg 3 — return drive to scoring position; score when done
    AutoTrajectory[] t1 = trajectories(routine, "RushToCenterUnoDip", 4);
    AutoTrajectory rush1 = t1[0];
    AutoTrajectory orient1 = t1[1];
    AutoTrajectory sweep1 = t1[2];
    AutoTrajectory returnToScore1 = t1[3];

    // DosDip has 4 segments (splits at samples 0, 20, 34, 143, targetDt=0.05 s):
    //   Seg 0 — fast rush from scoring position to midfield (~1.0 s);
    //           deploy intake here so it's fully extended before neutral zone
    //   Seg 1 — approach/pivot at neutral zone entrance (~0.7 s, PointAt ball cluster);
    //           intake continuing to deploy
    //   Seg 2 — full sweep through neutral zone (~5.45 s), covers both MaxVelocity=1.0 zones
    //           (WP4→5 and WP6→7); intake runs here — this is when ball #2 is collected
    //   Seg 3 — return to scoring position (~1.55 s); score when done
    AutoTrajectory[] t2 = trajectories(routine, "RushToCenterDosDip", 4);
    AutoTrajectory rush2 = t2[0];
    AutoTrajectory orient2 = t2[1];
    AutoTrajectory sweep2 = t2[2];
    AutoTrajectory returnToScore2 = t2[3];

    routine.active().onTrue(rush1.cmd());

    // Deploy intake during the orient pivot — gives it the full sweep to be ready
    rush1.done().onTrue(intake.deployCommand());
    rush1.done().onTrue(orient1.cmd());

    orient1.done().onTrue(sweep1.cmd());
    sweep1.active().whileTrue(intake.runCommand());

    sweep1.done().onTrue(returnToScore1.cmd());
    returnToScore1
        .done()
        .onTrue(
            actions
                .scoreWithReverseAgitationAndRollers()
                .withTimeout(5)
                .andThen(Commands.runOnce(rush2.cmd()::schedule)));

    // Deploy intake on the rush out — intake has the full orient+sweep (~6s) to be ready
    rush2.done().onTrue(intake.deployCommand());
    rush2.done().onTrue(orient2.cmd());

    orient2.done().onTrue(sweep2.cmd());
    sweep2.active().whileTrue(intake.runCommand());

    sweep2.done().onTrue(returnToScore2.cmd());
    returnToScore2.done().onTrue(actions.scoreWithReverseAgitationAndRollers().withTimeout(10));

    return routine;
  }

  public AutoRoutine singleGroundPickupAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Single Ground Pickup Auto");

    // UnoDip has 4 segments (splits at samples 0, 41, 52, 129):
    //   Seg 0 — fast rush along bottom to neutral zone entrance (~1.0 s, KeepInLane)
    //   Seg 1 — short orient pivot at neutral zone entrance (~0.3 s, PointAt ball cluster);
    //           deploy intake here so it's ready before the slow sweep begins
    //   Seg 2 — slow sweep through ball pickup zone (~1.9 s, MaxVelocity=1.0 m/s, PointAt);
    //           intake runs here — this is when the ball is collected
    //   Seg 3 — return drive to scoring position; score when done
    AutoTrajectory[] t = trajectories(routine, "RushToCenterUnoDip", 4);
    AutoTrajectory rush = t[0];
    AutoTrajectory orient = t[1];
    AutoTrajectory sweep = t[2];
    AutoTrajectory returnToScore = t[3];

    routine.active().onTrue(rush.cmd());

    rush.done().onTrue(intake.deployCommand());
    rush.done().onTrue(orient.cmd());

    orient.done().onTrue(sweep.cmd());
    sweep.active().whileTrue(intake.runCommand());

    sweep.done().onTrue(returnToScore.cmd());
    returnToScore.done().onTrue(actions.scoreWithReverseAgitationAndRollers().withTimeout(10));

    return routine;
  }

  public AutoRoutine oneDipLeftAuto() {
    AutoRoutine routine = autoFactory.newRoutine("One Dip Left Auto");

    // OneDipLeft has 4 segments (splits at samples 0, 8, 13, 32, targetDt=0.1 s):
    //   Seg 0 — fast rush along top of field to neutral zone entrance (~0.8 s, KeepInLane)
    //   Seg 1 — short orient approach at neutral zone entrance (~0.5 s, PointAt ball cluster);
    //           deploy intake here so it's ready before the slow sweep begins
    //   Seg 2 — slow sweep through ball pickup zone (~1.9 s, MaxVelocity=1.0 m/s, PointAt,
    //           KeepInLane); intake runs here — this is when the ball is collected
    //   Seg 3 — return drive to scoring position; score when done
    AutoTrajectory[] t = trajectories(routine, "OneDipLeft", 4);
    AutoTrajectory rush = t[0];
    AutoTrajectory orient = t[1];
    AutoTrajectory sweep = t[2];
    AutoTrajectory returnToScore = t[3];

    routine.active().onTrue(rush.cmd());

    rush.done().onTrue(intake.deployCommand());
    rush.done().onTrue(orient.cmd());

    orient.done().onTrue(sweep.cmd());
    sweep.active().whileTrue(intake.runCommand());

    sweep.done().onTrue(returnToScore.cmd());
    returnToScore.done().onTrue(actions.scoreWithReverseAgitationAndRollers().withTimeout(10));

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
