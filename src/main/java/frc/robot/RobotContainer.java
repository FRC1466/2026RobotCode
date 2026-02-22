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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
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
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
import frc.robot.subsystems.spindexer.SpindexerIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.TriggerUtil;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
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
  private Spindexer spindexer;
  private Kicker kicker;
  // private Choreographer choreographer;
  // private Autos autos;
  private Intake intake;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private DriveCommands driveCommand;

  private final Alert controllerDisconnected =
      new Alert("Controller disconnected (port 0).", AlertType.kWarning);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
          hood = new Hood(new HoodIO() {});
          spindexer = new Spindexer(new SpindexerIOTalonFX());
          kicker = new Kicker(new KickerIOTalonFX());
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
          flywheel = new Flywheel(new FlywheelIOSim());
          hood = new Hood(new HoodIOSim());
          spindexer = new Spindexer(new SpindexerIOSim());
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
    if (spindexer == null) {
      spindexer = new Spindexer(new SpindexerIO() {});
    }
    if (kicker == null) {
      kicker = new Kicker(new KickerIO() {});
    }
    if (intake == null) {
      intake = new Intake(new IntakePivotIO() {}, new IntakeRollersIO() {});
    }

    // Instantiate Choreographer
    // choreographer = new Choreographer(drive, flywheel, hood, spindexer, kicker, intake);

    // LoggedNetworkBoolean coastOverride =
    //     new LoggedNetworkBoolean("Choreographer/CoastOverride", false);
    // choreographer.setCoastOverride(coastOverride);

    // Set up Autos
    // autos = new Autos(drive, flywheel, hood, choreographer);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("None", Commands.none());

    // Add Choreo autos
    // autoChooser.addOption("Depot Auto (Choreo)", autos.depotAuto().cmd());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
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

    driveCommand = new DriveCommands(drive, controller);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(driveCommand);

    // --- Mutable state for manual tuning ---
    final double[] targetSpeed = {45.0};
    final boolean[] flywheelActive = {false};
    final double[] targetHoodAngle = {0.1};

    // Logging Command: Periodically logs target speed and hood angle to AdvantageScope
    RobotModeTriggers.teleop()
        .whileTrue(
            Commands.run(
                    () -> {
                      Logger.recordOutput("Debug/FlywheelTargetSpeed", targetSpeed[0]);
                      Logger.recordOutput("Debug/HoodTargetAngle", targetHoodAngle[0]);
                    })
                .withName("TuningLogger"));

    // --- Flywheel Controls ---

    // X Button: Toggle flywheel spinning at target speed
    controller
        .x()
        .onTrue(
            Commands.either(
                    flywheel.stopCommand(),
                    flywheel.runFixedCommand(() -> targetSpeed[0]),
                    () -> flywheelActive[0])
                .andThen(Commands.runOnce(() -> flywheelActive[0] = !flywheelActive[0])));

    // POV Left: Decrease flywheel target speed
    controller
        .povLeft()
        .onTrue(
            Commands.runOnce(() -> targetSpeed[0] = Math.max(0.0, targetSpeed[0] - 1.0))
                .withName("DecreaseFlywheelSpeed"));

    // POV Right: Increase flywheel target speed
    controller
        .povRight()
        .onTrue(
            Commands.runOnce(() -> targetSpeed[0] = Math.min(110.0, targetSpeed[0] + 1.0))
                .withName("IncreaseFlywheelSpeed"));

    // Left + Right Bumper: Toggle flywheel control mode (voltage / torque current)
    controller
        .leftBumper()
        .and(controller.rightBumper())
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (flywheel.getControlMode() == Flywheel.ControlMode.VOLTAGE) {
                        flywheel.setControlMode(Flywheel.ControlMode.TORQUE_CURRENT);
                      } else {
                        flywheel.setControlMode(Flywheel.ControlMode.VOLTAGE);
                      }
                    })
                .withName("ToggleFlywheelControlMode"));

    // --- Hood Controls ---

    // POV Up: Increase hood angle
    controller
        .povUp()
        .onTrue(
            Commands.runOnce(() -> targetHoodAngle[0] = Math.min(32.0, targetHoodAngle[0] + 1.0))
                .withName("IncreaseHoodAngle"));

    // POV Down: Decrease hood angle
    controller
        .povDown()
        .onTrue(
            Commands.runOnce(() -> targetHoodAngle[0] = Math.max(0.1, targetHoodAngle[0] - 1.0))
                .withName("DecreaseHoodAngle"));

    // Y Button: Move hood to target angle (hold)
    controller
        .y()
        .whileTrue(hood.runFixedCommand(() -> targetHoodAngle[0]).withName("HoodToTargetAngle"));

    // --- Intake Controls ---

    // A Button (single press): Deploy pivot and run rollers (hold)
    controller.a().whileTrue(intake.intakeCommand());

    // --- Utility Controls ---

    // Right Bumper (no Left): Launch-while-driving — auto-aims rotation at hub while driving
    controller
        .rightBumper()
        .and(controller.leftBumper().negate())
        .whileTrue(
            Commands.parallel(
                    driveCommand.launchModeCommand(),
                    flywheel.runTrackTargetCommand(),
                    hood.runTrackTargetCommand())
                .finallyDo(() -> ShotCalculator.getInstance().clearShootingParameters())
                .withName("DriveWhileLaunching"));

    // B Button: Stop all shooter subsystems (flywheel + hood)
    controller
        .b()
        .onTrue(
            Commands.parallel(flywheel.stopCommand(), hood.runFixedCommand(() -> 0.1))
                .andThen(Commands.runOnce(() -> flywheelActive[0] = false)));

    // Start + Back: Reset gyro
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

    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> HubShiftUtil.initialize()));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> HubShiftUtil.initialize()));
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
