// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ShooterConstants {
  public static final double reduction = 1;

  public static final double ks = 0.0;
  public static final LoggedTunableNumber kp =
      switch (Constants.robot) {
        case COMPBOT -> new LoggedTunableNumber("Shooter/kp", 0.0);
        case DEVBOT -> new LoggedTunableNumber("Shooter/kp", 15.0);
        default -> new LoggedTunableNumber("Shooter/kp", 0.0);
      };

  public static final LoggedTunableNumber kd =
      switch (Constants.robot) {
        case COMPBOT -> new LoggedTunableNumber("Shooter/kd", 0.0);
        case DEVBOT -> new LoggedTunableNumber("Shooter/kd", 0.0);
        default -> new LoggedTunableNumber("Shooter/kd", 0.0);
      };
}
