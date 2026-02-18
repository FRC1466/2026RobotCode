// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FlywheelIOSim implements FlywheelIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX60(2);
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, .025, 1), motorModel);

  private PIDController controller = new PIDController(0, 0, 0, Constants.loopPeriodSecs);
  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private double setpointRadsPerSec = 0.0;
  private double feedforward = 0.0;

  public FlywheelIOSim() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          controller.calculate(sim.getAngularVelocityRadPerSec(), setpointRadsPerSec) + feedforward;
    }

    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    // Update sim state
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.loopPeriodSecs);

    inputs.connectedMaster = true;
    inputs.connectedFollower = true;
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentMasterAmps = sim.getCurrentDrawAmps();
    inputs.supplyCurrentFollowerAmps = 0.0;
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
    inputs.tempMasterCelsius = 0.0;
    inputs.tempFollowerCelsius = 0.0;
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    if (outputs.mode == FlywheelIOOutputMode.VELOCITY_VOLTAGE
        || outputs.mode == FlywheelIOOutputMode.VELOCITY_TORQUE_CURRENT) {
      closedLoop = true;
      setpointRadsPerSec = outputs.velocityRadsPerSec;
      feedforward = outputs.feedforward;
      controller.setP(outputs.voltageKP);
      controller.setD(outputs.voltageKD);
    } else {
      closedLoop = false;
      appliedVolts = 0.0;
    }
  }
}
