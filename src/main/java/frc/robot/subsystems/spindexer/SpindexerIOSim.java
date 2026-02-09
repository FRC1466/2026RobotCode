// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SpindexerIOSim implements SpindexerIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX60(1);
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, 0.01, 1), motorModel);

  private double appliedVolts = 0.0;

  public SpindexerIOSim() {}

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.loopPeriodSecs);

    inputs.velocityRps = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;
    inputs.connected = true;
  }

  @Override
  public void applyOutputs(SpindexerIOOutputs outputs) {
    appliedVolts = outputs.appliedVolts;
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {}
}
