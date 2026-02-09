// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  class KickerIOInputs {
    public double velocityRps;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
    public boolean connected;
  }

  class KickerIOOutputs {
    public double appliedVolts = 0.0;
  }

  default void updateInputs(KickerIOInputs inputs) {}

  default void applyOutputs(KickerIOOutputs outputs) {}

  default void setBrakeMode(boolean enableBrake) {}
}
