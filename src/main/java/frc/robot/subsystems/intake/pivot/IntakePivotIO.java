// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {

  @AutoLog
  class IntakePivotIOInputs {
    public boolean motorConnected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  enum IntakePivotIOOutputMode {
    CLOSED_LOOP,
    OPEN_LOOP
  }

  class IntakePivotIOOutputs {
    public IntakePivotIOOutputMode mode = IntakePivotIOOutputMode.CLOSED_LOOP;
    public double positionRad = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
    public double volts = 0.0;
  }

  default void updateInputs(IntakePivotIOInputs inputs) {}

  default void applyOutputs(IntakePivotIOOutputs outputs) {}

  default void setBrakeMode(boolean enableBrake) {}
}
