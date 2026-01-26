// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FlywheelIOSim implements FlywheelIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX60(2);
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, .025, 1), motorModel);

  private PIDController controller = new PIDController(0, 0, 0, Constants.loopPeriodSecs);
  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private double setpointRps = 0.0;
  private double feedForward = 0.0;

  public FlywheelIOSim() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          controller.calculate(
                  Units.radiansToRotations(sim.getAngularVelocityRadPerSec()), setpointRps)
              + feedForward;
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
  public void applyOutputs(FlywheelIOOutputs outputs) {
    if (outputs.controlMode == FlywheelIOOutputs.ControlMode.VOLTAGE) {
      closedLoop = false;
      appliedVolts = outputs.appliedVolts;
    } else if (outputs.controlMode == FlywheelIOOutputs.ControlMode.DUTY_CYCLE_BANG_BANG) {
      closedLoop = false;
      if (Units.radiansToRotations(sim.getAngularVelocityRadPerSec()) < outputs.velocityRps) {
        appliedVolts = 12.0;
      } else {
        appliedVolts = 0.0;
      }
    } else {
      closedLoop = true;
    }
    setpointRps = outputs.velocityRps;
    feedForward =
        outputs.feedForward + (outputs.kS * Math.signum(setpointRps)) + (outputs.kV * setpointRps);
    controller.setP(outputs.kP);
    controller.setD(outputs.kD);
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {}
}
