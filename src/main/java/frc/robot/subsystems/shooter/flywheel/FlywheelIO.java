// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public boolean connectedMaster;
    public boolean connectedFollower;
    public double supplyCurrentMasterAmps;
    public double supplyCurrentFollowerAmps;
    public double tempMasterCelsius;
    public double tempFollowerCelsius;

    public double positionRotations;
    public double velocityRotationsPerSec;
    public double appliedVoltage;
    public double torqueCurrentAmps;
  }

  enum FlywheelIOOutputMode {
    COAST,
    VOLTAGE,
    VELOCITY_VOLTAGE,
    VELOCITY_TORQUE_CURRENT
  }

  class FlywheelIOOutputs {
    public FlywheelIOOutputMode mode = FlywheelIOOutputMode.COAST;
    public double velocityRotationsPerSec = 0.0;
    public double feedforward = 0.0;

    public double voltageKP;
    public double voltageKD;
    public double voltageKS;
    public double voltageKV;

    public double torqueCurrentKP;
    public double torqueCurrentKD;
    public double torqueCurrentKS;
    public double torqueCurrentKV;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}
}
