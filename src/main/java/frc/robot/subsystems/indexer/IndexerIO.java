// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  class IndexerIOInputs {
    public double velocityRps;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
    public boolean connected;
  }

  class IndexerIOOutputs {
    public double appliedVolts = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void applyOutputs(IndexerIOOutputs outputs) {}

  default void setBrakeMode(boolean enableBrake) {}
}
