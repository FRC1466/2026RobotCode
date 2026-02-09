// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Choreographer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;

public class Autos {
  private final Drive drive;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Choreographer choreographer;
  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;

  public Autos(Drive drive, Flywheel flywheel, Hood hood, Choreographer choreographer) {
    this.drive = drive;
    this.flywheel = flywheel;
    this.hood = hood;
    this.choreographer = choreographer;

    autoFactory =
        new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectory, true, drive);

    autoChooser = new AutoChooser();

    autoChooser.addRoutine("Depot Auto", this::depotAuto);

    autoChooser.addCmd("Do Nothing", () -> Commands.none());
  }

  public AutoChooser getAutoChooser() {
    return autoChooser;
  }

  public AutoRoutine depotAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Depot Auto");

    AutoTrajectory depotTraj = routine.trajectory("depotAuto");

    routine.active().onTrue(Commands.sequence(depotTraj.resetOdometry(), depotTraj.cmd()));

    return routine;
  }
}
