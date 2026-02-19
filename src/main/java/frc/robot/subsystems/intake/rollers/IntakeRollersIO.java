// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
  @AutoLog
  class IntakeRollersIOInputs {
    public double velocityRps;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
    public boolean connected;
  }

  class IntakeRollersIOOutputs {
    public double appliedVolts = 0.0;
  }

  default void updateInputs(IntakeRollersIOInputs inputs) {}

  default void applyOutputs(IntakeRollersIOOutputs outputs) {}

  default void setBrakeMode(boolean enableBrake) {}
}
