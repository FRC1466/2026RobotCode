// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    boolean motorConnected = false;
    double positionRads = 0.0;
    double velocityRadsPerSec = 0.0;
    double appliedVolts = 0.0;
    double supplyCurrentAmps = 0.0;
    double torqueCurrentAmps = 0.0;
    double tempCelsius = 0.0;
  }

  enum HoodIOOutputMode {
    CLOSED_LOOP,
    OPEN_LOOP
  }

  public static class HoodIOOutputs {
    public HoodIOOutputMode mode = HoodIOOutputMode.CLOSED_LOOP;
    public double positionRad = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
    public double volts = 0.0; // Used for Open Loop only
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void applyOutputs(HoodIOOutputs outputs) {}

  default void setBrakeMode(boolean enableBrake) {}
}
