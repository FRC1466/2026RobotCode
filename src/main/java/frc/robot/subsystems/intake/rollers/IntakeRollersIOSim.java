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
  // TODO: Change to 1 motor when hardware is updated.
  private static final DCMotor motorModel = DCMotor.getKrakenX44(2);
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, 0.005, 1), motorModel);

  private double appliedVolts = 0.0;

  public IntakeRollersIOSim() {}

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
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
  public void applyOutputs(IntakeRollersIOOutputs outputs) {
    appliedVolts = outputs.appliedVolts;
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {}
}
