// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Choreographer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.pivot.IntakePivotIO;
import frc.robot.subsystems.intake.pivot.IntakePivotIOSim;
import frc.robot.subsystems.intake.pivot.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.rollers.IntakeRollersIO;
import frc.robot.subsystems.intake.rollers.IntakeRollersIOSim;
import frc.robot.subsystems.intake.rollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.subsystems.kicker.KickerIOTalonFX;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TriggerUtil;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@ExtensionMethod({TriggerUtil.class})
public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;
  private Flywheel flywheel;
  private Hood hood;
  private Indexer indexer;
  private Kicker kicker;
  private Choreographer choreographer;
  private Autos autos;
  private Intake intake;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private DriveCommands driveCommand;

  private final Alert controllerDisconnected =
      new Alert("Controller disconnected (port 0).", AlertType.kWarning);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Tuning values — editable live via NetworkTables when tuningMode=true, otherwise fixed defaults.
  // Visible under /Tuning/ in AdvantageScope / NT explorer.
  private static final LoggedTunableNumber manualFlywheelSpeed =
      new LoggedTunableNumber("Shooter/ManualFlywheelSpeedRPS", 45.0);
  private static final LoggedTunableNumber manualHoodAngle =
      new LoggedTunableNumber("Shooter/ManualHoodAngleDeg", 0.1);
  private static final LoggedTunableNumber manualIntakeDeployAngle =
      new LoggedTunableNumber("Intake/ManualDeployAngleDeg", 105.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.robot) {
        case COMPBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVision(camera0Name, robotToCamera0));

          flywheel = new Flywheel(new FlywheelIOTalonFX());
          hood = new Hood(new HoodIOTalonFX());
          indexer = new Indexer(new IndexerIOTalonFX());
          kicker = new Kicker(new KickerIOTalonFX());
          intake = new Intake(new IntakePivotIOTalonFX(), new IntakeRollersIOTalonFX());
          break;
        }
        case DEVBOT -> {
          /*drive =
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOTalonFX(TunerConstants.FrontLeft),
              new ModuleIOTalonFX(TunerConstants.FrontRight),
              new ModuleIOTalonFX(TunerConstants.BackLeft),
              new ModuleIOTalonFX(TunerConstants.BackRight));*/
          flywheel = new Flywheel(new FlywheelIOTalonFX());
          hood = new Hood(new HoodIOTalonFX());
          intake = new Intake(new IntakePivotIOTalonFX(), new IntakeRollersIOTalonFX());
          break;
        }
        case SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(TunerConstants.FrontLeft),
                  new ModuleIOSim(TunerConstants.FrontRight),
                  new ModuleIOSim(TunerConstants.BackLeft),
                  new ModuleIOSim(TunerConstants.BackRight));
          vision =
              new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));
          flywheel = new Flywheel(new FlywheelIOSim());
          hood = new Hood(new HoodIOSim());
          indexer = new Indexer(new IndexerIOSim());
          kicker = new Kicker(new KickerIOSim());
          intake = new Intake(new IntakePivotIOSim(), new IntakeRollersIOSim());
          break;
        }
      }
    }

    // No-op implementations for replay or if not set above
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (vision == null) {
      vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
    }
    if (flywheel == null) {
      flywheel = new Flywheel(new FlywheelIO() {});
    }
    if (hood == null) {
      hood = new Hood(new HoodIO() {});
    }
    if (indexer == null) {
      indexer = new Indexer(new IndexerIO() {});
    }
    if (kicker == null) {
      kicker = new Kicker(new KickerIO() {});
    }
    if (intake == null) {
      intake = new Intake(new IntakePivotIO() {}, new IntakeRollersIO() {});
    }

    // Instantiate Choreographer
    choreographer = new Choreographer(drive, flywheel, hood, indexer, kicker, intake);

    // Set up Autos
    autos = new Autos(drive, flywheel, hood, choreographer);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("None", Commands.none());

    // Add Choreo autos
    autoChooser.addOption("Depot Auto (Choreo)", autos.depotAuto().cmd());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Subsystem bring-up tests — run each motor at a fixed voltage for verification
    autoChooser.addOption(
        "Test: Flywheel 3V",
        flywheel.runVolts(() -> 3.0).withTimeout(5.0).withName("Test Flywheel 3V"));
    autoChooser.addOption(
        "Test: Hood", hood.runVolts(() -> 0.5).withTimeout(3.0).withName("Test Hood 0.5V"));
    autoChooser.addOption(
        "Test: Intake Pivot",
        intake.runVolts(() -> 0.25).withTimeout(3.0).withName("Test Intake Pivot 0.25V"));
    autoChooser.addOption(
        "Test: Intake Rollers",
        intake.runCommand().withTimeout(3.0).withName("Test Intake Rollers"));
    autoChooser.addOption(
        "Test: Indexer", indexer.runCommand().withTimeout(3.0).withName("Test Indexer"));
    autoChooser.addOption(
        "Test: Kicker", kicker.runCommand().withTimeout(3.0).withName("Test Kicker"));

    driveCommand = new DriveCommands(drive, controller);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   *
   * <p>Layout:
   *
   * <pre>
   *  RIGHT TRIGGER  — Choreographer SCORE_HUB + auto-rotate drive
   *  LEFT TRIGGER   — Choreographer INTAKE
   *  LEFT BUMPER    — Speed boost (hold)
   *  RIGHT BUMPER   — (reserved / future climb)
   *  A              — Emergency stop → Choreographer IDLE
   *  B              — (reserved)
   *  X              — Tuning: spin flywheel at manualFlywheelSpeed (hold, Choreographer disabled)
   *  Y              — Tuning: move hood to manualHoodAngle (hold, Choreographer disabled)
   *  POV LEFT/RIGHT — Tuning: adjust flywheel speed ±1 RPS
   *  POV UP/DOWN    — Tuning: adjust hood angle ±1°
   *  POV UP+RIGHT   — Tuning: intake deploy angle +1°
   *  POV DOWN+LEFT  — Tuning: intake deploy angle -1°
   *  BACK (solo)    — Toggle Choreographer enabled/disabled (enter/exit tuning mode)
   *  START + BACK   — Reset gyro to forward
   * </pre>
   */
  private void configureButtonBindings() {
    // Default command — normal field-relative drive
    drive.setDefaultCommand(driveCommand);

    // ── Drive overrides ──────────────────────────────────────────────────────

    // Left Bumper: slow mode (hold) — default is 4.5 m/s, slow to 3 m/s
    controller.leftBumper().whileTrue(driveCommand.slowDownCommand());

    // Right Bumper: toggle hub-preset override — bypasses vision/pose, uses hubPreset values and
    // the robot's current heading (no auto-rotation). Visible on SmartDashboard as
    // "ShotCalculator/HubPresetOverride". Use when vision is unreliable and you know your position.
    controller
        .rightBumper()
        .onTrue(
            Commands.runOnce(() -> ShotCalculator.getInstance().toggleHubPresetOverride())
                .withName("ToggleHubPresetOverride")
                .ignoringDisable(true));

    // Start + Back: reset gyro heading to alliance-forward
    controller
        .start()
        .and(controller.back())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                AllianceFlipUtil.apply(Rotation2d.kZero))))
                .withName("ResetGyro")
                .ignoringDisable(true));

    // ── Primary scoring (Choreographer) ──────────────────────────────────────

    // Right Trigger: SCORE_HUB — Choreographer handles flywheel, hood, indexer, kicker.
    // Drive auto-rotates to target while held.
    controller
        .rightTrigger()
        .onTrue(choreographer.setGoalCommand(Choreographer.Goal.SCORE_HUB))
        .whileTrue(driveCommand.launchModeCommand())
        .onFalse(choreographer.setGoalCommand(Choreographer.Goal.IDLE));

    // Left Trigger: INTAKE — Choreographer deploys pivot and runs rollers.
    controller
        .leftTrigger()
        .onTrue(choreographer.setGoalCommand(Choreographer.Goal.INTAKE))
        .onFalse(choreographer.setGoalCommand(Choreographer.Goal.IDLE));

    // ── Emergency stop ────────────────────────────────────────────────────────

    // A Button: cancel everything → Choreographer IDLE (stows intake, stops shooter)
    controller
        .a()
        .onTrue(choreographer.setGoalCommand(Choreographer.Goal.IDLE).withName("EmergencyStop"));

    // ── Tuning mode (active only when Choreographer is disabled via Back button) ──

    // Back (solo): toggle Choreographer enabled — when disabled, X/Y + POV tune directly
    controller.back().and(controller.start().negate()).onTrue(choreographer.toggleEnabledCommand());

    // B Button: hold to deploy intake to manualIntakeDeployAngle (tuning, Choreographer off)
    controller
        .b()
        .whileTrue(
            intake.runFixedCommand(manualIntakeDeployAngle).withName("TuneIntakeDeployAngle"))
        .onFalse(intake.stowCommand());

    // X Button: hold to spin flywheel at manualFlywheelSpeed (tuning, Choreographer off)
    controller
        .x()
        .whileTrue(flywheel.runFixedCommand(manualFlywheelSpeed).withName("TuneFlywheelSpin"))
        .onFalse(flywheel.stopCommand());

    // Y Button: hold to move hood to manualHoodAngle (tuning, Choreographer off)
    controller
        .y()
        .whileTrue(hood.runFixedCommand(manualHoodAngle).withName("TuneHoodAngle"))
        .onFalse(hood.stowCommand());

    // ── Mode init ────────────────────────────────────────────────────────────
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
  }

  /** Update dashboard outputs. */
  public void updateDashboardOutputs() {
    // Publish match time
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Controller disconnected alerts
    controllerDisconnected.set(!DriverStation.isJoystickConnected(controller.getHID().getPort()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
