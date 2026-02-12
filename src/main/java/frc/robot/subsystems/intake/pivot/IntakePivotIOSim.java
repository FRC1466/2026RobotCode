// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class IntakePivotIOSim implements IntakePivotIO {
  private static final double gearing = 1.0; // TODO: Set correct gear ratio.
  private static final double moi = 0.15; // kg * m^2
  private static final double armLength = 0.4; // m
  private static final double minAngle = Units.degreesToRadians(0.0);
  private static final double maxAngle = Units.degreesToRadians(120.0);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1), gearing, moi, armLength, minAngle, maxAngle, false, minAngle);

  private final PIDController controller =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);

  private double appliedVolts = 0.0;
  private boolean closedLoop = false;

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = MathUtil.clamp(controller.calculate(sim.getAngleRads()), -12.0, 12.0);
    }

    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.loopPeriodSecs);

    double position = MathUtil.clamp(sim.getAngleRads(), minAngle, maxAngle);

    inputs.motorConnected = true;
    inputs.positionRads = position;
    inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(IntakePivotIOOutputs outputs) {
    if (outputs.mode == IntakePivotIOOutputMode.OPEN_LOOP) {
      closedLoop = false;
      appliedVolts = MathUtil.clamp(outputs.volts, -12.0, 12.0);
    } else if (outputs.mode == IntakePivotIOOutputMode.CLOSED_LOOP) {
      closedLoop = true;
      controller.setPID(outputs.kP, 0.0, outputs.kD);
      controller.setSetpoint(outputs.positionRad);
    }
  }
}
