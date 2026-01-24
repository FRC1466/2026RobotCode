// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.TriggerUtil;
import java.util.function.DoubleSupplier;
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

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

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
          hood = new Hood(new HoodIO() {}); // TODO: Implement HoodIOTalonFX/Sim
          break;
        }
        case DEVBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
          // flywheel = new Flywheel(new FlywheelIOTalonFX());
          // hood = new Hood(new HoodIO() {});
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
          hood = new Hood(new HoodIO() {});
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

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
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
    DoubleSupplier leftY = () -> -controller.getLeftY();
    DoubleSupplier leftX = () -> -controller.getLeftX();
    DoubleSupplier rightX = () -> -controller.getRightX();

    // Default command, normal field-relative drive
    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, leftY, leftX, rightX));

    flywheel.setDefaultCommand(flywheel.stopCommand());
    hood.setDefaultCommand(hood.runTrackTargetCommand());

    // Right Bumper: Auto-aim robot to target and rev shooter (flywheel + hood)
    controller
        .rightBumper()
        .whileTrue(
            Commands.parallel(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    leftY,
                    leftX,
                    () -> ShotCalculator.getInstance().getParameters().goalHeading()),
                flywheel.runTrackTargetCommand(),
                hood.runTrackTargetCommand()));

    controller.a().whileTrue(flywheel.runTrackTargetCommand());

    // POV: Manual control / preset speeds
    controller.povUp().whileTrue(flywheel.runFixedCommand(() -> 105)); // Near max
    controller.povDown().whileTrue(flywheel.runFixedCommand(() -> 45)); // Min range

    // Left Bumper: Zero hood
    controller.leftBumper().onTrue(hood.zeroCommand());

    controller
        .rightStick()
        .whileTrue(
            drive.defer(
                () ->
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        leftY,
                        leftX,
                        () -> {
                          Rotation2d currentRotation = RobotState.getInstance().getRotation();
                          // Snap to nearest 45 degrees, excluding 0, 90, 180, 270
                          double angle = currentRotation.getDegrees();
                          double snapped = Math.round(angle / 45.0) * 45.0;
                          // Adjust if snapped to cardinal direction
                          if (snapped % 90.0 == 0.0) {
                            snapped += (angle > snapped) ? 45.0 : -45.0;
                          }
                          return Rotation2d.fromDegrees(snapped);
                        })))
        .onTrue(Commands.runOnce(() -> vision.setRampMode(true)).withName("EnableRampMode"))
        .onFalse(Commands.runOnce(() -> vision.setRampMode(false)).withName("DisableRampMode"));

    // Mutable state for manual flywheel control
    final double[] targetSpeed = {45.0};
    final boolean[] flywheelActive = {false};

    // Logging Command: Periodically logs distance and target speed to AdvantageScope
    Commands.run(
            () -> {
              Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
              if (currentPose != null) {
                Translation2d hubPose = AllianceFlipUtil.apply(FieldConstants.hubCenter);
                double distance = currentPose.getTranslation().getDistance(hubPose);
                org.littletonrobotics.junction.Logger.recordOutput("Debug/DistanceToHub", distance);
              }
              org.littletonrobotics.junction.Logger.recordOutput(
                  "Debug/FlywheelTargetSpeed", targetSpeed[0]);
            })
        .ignoringDisable(true)
        .withName("DebugLogger")
        .schedule();

    // X Button: Toggle flywheel spinning
    controller
        .x()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      flywheelActive[0] = !flywheelActive[0];
                      if (flywheelActive[0]) {
                        // Supplier ensures speed updates dynamically without rescheduling
                        flywheel.runFixedCommand(() -> targetSpeed[0]).schedule();
                      } else {
                        flywheel.stopCommand().schedule();
                      }
                    })
                .withName("ToggleFlywheel"));

    // POV Left: Decrease target speed
    controller
        .povLeft()
        .onTrue(
            Commands.runOnce(() -> targetSpeed[0] = Math.max(30.0, targetSpeed[0] - 1.0))
                .withName("DecreaseSpeed"));

    // POV Right: Increase target speed
    controller
        .povRight()
        .onTrue(
            Commands.runOnce(() -> targetSpeed[0] = Math.min(110.0, targetSpeed[0] + 1.0))
                .withName("IncreaseSpeed"));

    // B Button: Stop all shooter subsystems
    controller
        .b()
        .onTrue(
            Commands.parallel(
                flywheel.stopCommand(),
                hood.runFixedCommand(() -> Units.degreesToRadians(19), () -> 0.0)));

    // Reset gyro
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
