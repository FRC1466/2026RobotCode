// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeRollersIOSim implements IntakeRollersIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX44(1);
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, 0.005, 1), motorModel);

  private double appliedVolts = 0.0;
  private double velocityRpsSetpoint = 0.0;
  private IntakeRollersOutputMode mode = IntakeRollersOutputMode.OPEN_LOOP;

  public IntakeRollersIOSim() {}

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
    double inputVolts = appliedVolts;
    if (mode == IntakeRollersOutputMode.VELOCITY_PID) {
      // Simple proportional controller for sim
      double currentRps = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
      double error = velocityRpsSetpoint - currentRps;
      inputVolts = MathUtil.clamp(error * 6.0, -12.0, 12.0); // P gain = 6.0
    }
    inputVolts = MathUtil.clamp(inputVolts, -12.0, 12.0);
    sim.setInputVoltage(inputVolts);
    sim.update(Constants.loopPeriodSecs);

    inputs.velocityRps = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
    inputs.appliedVoltage = inputVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;
    inputs.connected = true;
  }

  @Override
  public void applyOutputs(IntakeRollersIOOutputs outputs) {
    this.mode = outputs.mode;
    if (outputs.mode == IntakeRollersOutputMode.VELOCITY_PID) {
      this.velocityRpsSetpoint = outputs.velocityRpsSetpoint;
    } else {
      this.appliedVolts = outputs.appliedVolts;
    }
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {}
}
