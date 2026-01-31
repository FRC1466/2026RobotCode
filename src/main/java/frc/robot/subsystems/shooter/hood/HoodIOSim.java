// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class HoodIOSim implements HoodIO {
  private static final double gearing = 8.0;
  private static final double moi = 0.001; // kg * m^2 (smaller, more responsive)
  private static final double minAngle = Units.degreesToRadians(19.0);
  private static final double maxAngle = Units.degreesToRadians(51.0);

  private static final DCMotor motorModel = DCMotor.getKrakenX44(1);
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, gearing), motorModel);

  private final PIDController controller =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);

  private double appliedVolts = 0.0;
  private boolean closedLoop = false;

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = MathUtil.clamp(controller.calculate(sim.getAngularPositionRad()), -12.0, 12.0);
    }

    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.loopPeriodSecs);

    // Clamp position to physical limits
    double position = MathUtil.clamp(sim.getAngularPositionRad(), minAngle, maxAngle);

    inputs.motorConnected = true;
    inputs.positionRads = position;
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    if (outputs.mode == HoodIOOutputMode.OPEN_LOOP) {
      closedLoop = false;
      appliedVolts = MathUtil.clamp(outputs.volts, -12.0, 12.0);
    } else if (outputs.mode == HoodIOOutputMode.CLOSED_LOOP) {
      closedLoop = true;
      controller.setPID(outputs.kP, 0.0, outputs.kD);
      controller.setSetpoint(outputs.positionRad);
    }
  }
}
