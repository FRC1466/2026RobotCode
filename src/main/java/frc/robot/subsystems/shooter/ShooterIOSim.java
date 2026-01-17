// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX60(1);
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, .025, 1), motorModel);

  private PIDController controller = new PIDController(0, 0, 0, Constants.loopPeriodSecs);
  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private double setpointRps = 0.0;
  private double feedForward = 0.0;

  public ShooterIOSim() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          controller.calculate(
                  Units.radiansToRotations(sim.getAngularVelocityRadPerSec()), setpointRps)
              + feedForward;
    } else {
      appliedVolts = 0.0;
    }

    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    // Update sim state
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.loopPeriodSecs);

    inputs.positionRotations = Units.radiansToRotations(sim.getAngularPositionRad());
    inputs.velocityRps = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;
    inputs.connected = true;
  }

  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    closedLoop = !outputs.coast;
    setpointRps = outputs.velocityRps;
    feedForward =
        outputs.feedForward + (outputs.kS * Math.signum(setpointRps)) + (outputs.kV * setpointRps);
    controller.setP(outputs.kP);
    controller.setD(outputs.kD);
  }
}
