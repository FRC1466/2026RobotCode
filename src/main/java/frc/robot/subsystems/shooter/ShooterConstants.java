// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter;

import frc.robot.util.LoggedTunableNumber;

public class ShooterConstants {
  public static final double reduction = 1;

  public static final LoggedTunableNumber ks = new LoggedTunableNumber("Shooter/ks", 0.19);
  public static final LoggedTunableNumber kv = new LoggedTunableNumber("Shooter/kv", 0.11);
  public static final LoggedTunableNumber kp = new LoggedTunableNumber("Shooter/kp", 0.3);
  public static final LoggedTunableNumber kd = new LoggedTunableNumber("Shooter/kd", 0.0);
}
