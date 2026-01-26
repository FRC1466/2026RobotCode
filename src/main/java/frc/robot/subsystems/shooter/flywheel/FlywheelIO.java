// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public double positionRotations;
    public double velocityRps;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
    public boolean connected;
  }

  class FlywheelIOOutputs {
    public enum ControlMode {
      VELOCITY,
      VOLTAGE,
      DUTY_CYCLE_BANG_BANG
    }

    public ControlMode controlMode = ControlMode.VELOCITY;
    public double velocityRps = 0.0;
    public double feedForward = 0.0;
    public double appliedVolts = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
    public double kS = 0.0;
    public double kV = 0.0;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}

  default void setBrakeMode(boolean enableBrake) {}
}
