// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final RobotType robot = RobotType.SIMBOT;
  public static final boolean tuningMode = false;

  public static final double loopPeriodSecs = 0.02;
  public static final double loopPeriodWatchdogSecs = 0.2;

  public static Mode getMode() {
    return switch (robot) {
      case COMPBOT, DEVBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    COMPBOT,
    DEVBOT,
    SIMBOT
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robot == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robot);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robot != RobotType.COMPBOT || tuningMode) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }

  public static class ControllerConstants {
    public static final double CONTROLLER_DEADBAND = 0.225;
    public static final double CONTROLLER_RUMBLE = 0.3;
  }

  public static class FieldConstants {
    public static final Distance FIELD_LENGTH = Inches.of(650.12);
    public static final Distance FIELD_WIDTH = Inches.of(316.64);

    public static final Distance TRENCH_BUMP_X = Inches.of(181.56);
    public static final Distance TRENCH_WIDTH = Inches.of(49.86);
    private static final Distance BUMP_INSET = TRENCH_WIDTH.plus(Inches.of(12));
    private static final Distance BUMP_LENGTH = Inches.of(73);

    private static final Distance TRENCH_ZONE_EXTENSION = Inches.of(60);
    private static final Distance BUMP_ZONE_EXTENSION = Inches.of(60);
    private static final Distance TRENCH_BUMP_ZONE_TRANSITION =
        TRENCH_WIDTH.plus(BUMP_INSET).div(2);

    public static final Translation2d[][] TRENCH_ZONES = {
      new Translation2d[] {
        new Translation2d(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION).in(Inches) * 0.0254, 0.0),
        new Translation2d(
            TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION).in(Inches) * 0.0254,
            TRENCH_BUMP_ZONE_TRANSITION.in(Inches) * 0.0254)
      },
      new Translation2d[] {
        new Translation2d(
            TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION).in(Inches) * 0.0254,
            FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION).in(Inches) * 0.0254),
        new Translation2d(
            TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION).in(Inches) * 0.0254,
            FIELD_WIDTH.in(Inches) * 0.0254)
      },
      new Translation2d[] {
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION)).in(Inches) * 0.0254, 0.0),
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION)).in(Inches) * 0.0254,
            TRENCH_BUMP_ZONE_TRANSITION.in(Inches) * 0.0254)
      },
      new Translation2d[] {
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION)).in(Inches) * 0.0254,
            FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION).in(Inches) * 0.0254),
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION)).in(Inches) * 0.0254,
            FIELD_WIDTH.in(Inches) * 0.0254)
      }
    };

    public static final Translation2d[][] BUMP_ZONES = {
      new Translation2d[] {
        new Translation2d(
            TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION).in(Inches) * 0.0254,
            TRENCH_BUMP_ZONE_TRANSITION.in(Inches) * 0.0254),
        new Translation2d(
            TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION).in(Inches) * 0.0254,
            BUMP_INSET.plus(BUMP_LENGTH).in(Inches) * 0.0254)
      },
      new Translation2d[] {
        new Translation2d(
            TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION).in(Inches) * 0.0254,
            FIELD_WIDTH.minus(BUMP_INSET.plus(BUMP_LENGTH)).in(Inches) * 0.0254),
        new Translation2d(
            TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION).in(Inches) * 0.0254,
            FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION).in(Inches) * 0.0254)
      },
      new Translation2d[] {
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION)).in(Inches) * 0.0254,
            FIELD_WIDTH.minus(BUMP_INSET.plus(BUMP_LENGTH)).in(Inches) * 0.0254),
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION)).in(Inches) * 0.0254,
            FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION).in(Inches) * 0.0254)
      },
      new Translation2d[] {
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION)).in(Inches) * 0.0254,
            TRENCH_BUMP_ZONE_TRANSITION.in(Inches) * 0.0254),
        new Translation2d(
            FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION)).in(Inches) * 0.0254,
            BUMP_INSET.plus(BUMP_LENGTH).in(Inches) * 0.0254)
      }
    };

    public static final Distance TRENCH_CENTER = TRENCH_WIDTH.div(2);
  }
}
