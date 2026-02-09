// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.util.PhoenixUtil;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX talon;
  
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  private final PositionVoltage request = new PositionVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private double lastKp = 0.0;
  private double lastKd = 0.0;

  public HoodIOTalonFX() {
    talon = new TalonFX(15);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.RotorToSensorRatio = 8.0;
    config.Feedback.SensorToMechanismRatio = 1.0;

    config.Slot0.kP = 0.0;
    config.Slot0.kD = 0.0;

    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp));
    PhoenixUtil.tryUntilOk(5, () -> talon.optimizeBusUtilization());
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp);
    inputs.motorConnected =
        BaseStatusSignal.isAllGood(
            position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp);
    inputs.positionRads = position.getValue().in(Radians);
    inputs.velocityRadsPerSec = velocity.getValue().in(RadiansPerSecond);
    inputs.appliedVolts = appliedVoltage.getValue().in(Volts);
    inputs.supplyCurrentAmps = supplyCurrent.getValue().in(Amps);
    inputs.torqueCurrentAmps = torqueCurrent.getValue().in(Amps);
    inputs.tempCelsius = temp.getValue().in(Celsius);
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {
    talon.setNeutralMode(enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    if (outputs.mode == HoodIOOutputMode.OPEN_LOOP) {
      talon.setControl(voltageRequest.withOutput(outputs.volts));
    } else if (outputs.mode == HoodIOOutputMode.CLOSED_LOOP) {
      if (outputs.kP != lastKp || outputs.kD != lastKd) {
        var slot0 = new Slot0Configs();
        slot0.kP = outputs.kP;
        slot0.kD = outputs.kD;

        PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(slot0));
        lastKp = outputs.kP;
        lastKd = outputs.kD;
      }
      talon.setControl(request.withPosition(Rotations.of(outputs.positionRad / (2 * Math.PI))));
    }
  }
}
