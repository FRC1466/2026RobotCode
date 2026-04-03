// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  // CAD origin: +X=left, +Y=backward, +Z=down
  // WPILib: X=forward(-cadY), Y=left(+cadX), Z=up(-cadZ)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(3.977574),
          Units.inchesToMeters(-9.194390),
          Units.inchesToMeters(20.073830),
          new Rotation3d(0.0, Units.degreesToRadians(-70), 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(.842),
          Units.inchesToMeters(9.189),
          Units.inchesToMeters(14.067),
          new Rotation3d(Math.PI, Units.degreesToRadians(70), Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.1; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
