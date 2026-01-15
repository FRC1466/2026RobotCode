// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
    public boolean connected;
  }

  public static class ShooterIOOutputs {
    public double velocityRadsPerSec = 0.0;
    public double feedForward = 0.0;
    public boolean coast = true;
    public double kP = 0.0;
    public double kD = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void applyOutputs(ShooterIOOutputs outputs) {}
}
