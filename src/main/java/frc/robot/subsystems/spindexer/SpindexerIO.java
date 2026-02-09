// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  class SpindexerIOInputs {
    public double velocityRps;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
    public boolean connected;
  }

  class SpindexerIOOutputs {
    public double appliedVolts = 0.0;
  }

  default void updateInputs(SpindexerIOInputs inputs) {}

  default void applyOutputs(SpindexerIOOutputs outputs) {}

  default void setBrakeMode(boolean enableBrake) {}
}
