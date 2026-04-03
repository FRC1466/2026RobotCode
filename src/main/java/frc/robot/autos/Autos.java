// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.autos;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.util.AllianceFlipUtil;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final AutoChooser autoChooser;
  private final AutoRoutines routines;

  private final Map<String, Supplier<Optional<Pose2d>>> autoStartPoseSuppliers = new HashMap<>();
  private final Map<String, Supplier<Optional<Pose2d>>> autoStartPoseSuppliersByCommandName =
      new HashMap<>();

  private final edu.wpi.first.wpilibj.smartdashboard.Field2d autoStartField =
      new edu.wpi.first.wpilibj.smartdashboard.Field2d();

  public Autos(RobotContainer robotContainer) {
    AutoFactory autoFactory =
        new AutoFactory(
            robotContainer.getDrive()::getPose,
            robotContainer.getDrive()::setPose,
            robotContainer.getDrive()::followTrajectory,
            true,
            robotContainer.getDrive());

    AutoActions actions =
        new AutoActions(
            robotContainer.getChoreographer(), robotContainer.getIntake(), robotContainer);

    routines = new AutoRoutines(robotContainer, autoFactory, actions);

    autoChooser = new AutoChooser();

    autoStartPoseSuppliers.put(autoChooser.getDefaultName(), Optional::<Pose2d>empty);
    autoStartPoseSuppliersByCommandName.put(autoChooser.getDefaultName(), Optional::<Pose2d>empty);

    // ─── Register Autos ─────────────────────────────────────────────

    registerRoutine(
        "Outpost Auto",
        routines::outpostAuto,
        () -> routines.startPoseOf(routines::outpostAuto, r -> r.trajectory("OutpostAuto", 0)));

    registerRoutine(
        "Preload Then Outpost Auto",
        routines::preloadThenOutpostAuto,
        () ->
            routines.startPoseOf(
                routines::preloadThenOutpostAuto, r -> r.trajectory("OutpostGroundPreload", 0)));

    registerRoutine(
        "Ground Auto",
        routines::groundAuto,
        () -> routines.startPoseOf(routines::groundAuto, r -> r.trajectory("GrabFromGround", 0)));

    registerRoutine(
        "Preload Then Ground Auto",
        routines::preloadThenGroundAuto,
        () ->
            routines.startPoseOf(
                routines::preloadThenGroundAuto, r -> r.trajectory("GrabFromGround", 0)));

    registerRoutine(
        "Double Ground Pickup Auto",
        routines::doubleGroundPickupAuto,
        () ->
            routines.startPoseOf(
                routines::doubleGroundPickupAuto, r -> r.trajectory("RushToCenterUnoDip", 0)));

    registerRoutine(
        "Single Ground Pickup Auto",
        routines::singleGroundPickupAuto,
        () ->
            routines.startPoseOf(
                routines::singleGroundPickupAuto, r -> r.trajectory("RushToCenterUnoDip", 0)));

    registerRoutine(
        "Drive Back Preload Auto",
        routines::driveBackPreloadAuto,
        () ->
            Optional.of(
                AllianceFlipUtil.apply(
                    new Pose2d(3.5, FieldConstants.fieldWidth / 2.0, Rotation2d.kZero))));

    registerRoutine(
        "Drive Left Preload Auto",
        routines::driveLeftPreloadAuto,
        () -> routines.startPoseOf(routines::driveLeftPreloadAuto, r -> r.trajectory("LeftAuto")));

    registerRoutine("Left Preload Auto", routines::LeftPreloadAuto, Optional::<Pose2d>empty);

    registerRoutine(
        "One Dip Left Auto",
        routines::oneDipLeftAuto,
        () -> routines.startPoseOf(routines::oneDipLeftAuto, r -> r.trajectory("OneDipLeft", 0)));
    registerRoutine(
        "Bumpy",
        routines::bumpy,
        () -> routines.startPoseOf(routines::bumpy, r -> r.trajectory("Bumpy", 0)));

    // ─── Dashboard ─────────────────────────────────────────────────

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Auto Start Pose", autoStartField);

    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  public AutoChooser getAutoChooser() {
    return autoChooser;
  }

  public void updateDashboardOutputs() {
    if (!DriverStation.isDisabled()) return;

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
        "Autos/StartPose", startPose.map(p -> new Pose2d[] {p}).orElseGet(() -> new Pose2d[0]));
  }

  private void registerRoutine(
      String name, Supplier<AutoRoutine> supplier, Supplier<Optional<Pose2d>> startPoseSupplier) {

    autoChooser.addRoutine(name, supplier);
    autoStartPoseSuppliers.put(name, startPoseSupplier);
    autoStartPoseSuppliersByCommandName.put(name, startPoseSupplier);

    // Also map command name → start pose
    AutoRoutine preview = supplier.get();
    autoStartPoseSuppliersByCommandName.put(preview.cmd().getName(), startPoseSupplier);
  }
}
