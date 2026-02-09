// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.util.PhoenixUtil;

public class KickerIOTalonFX implements KickerIO {
  // TODO: Move CAN ID into constants.
  private static final int motorId = 16;

  private final TalonFX talon;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  private final VoltageOut voltageRequest = new VoltageOut(0);

  public KickerIOTalonFX() {
    talon = new TalonFX(motorId);

    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 30.0;

    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));

    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp));
    PhoenixUtil.tryUntilOk(5, () -> talon.optimizeBusUtilization());
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {
    talon.setNeutralMode(enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp);
    inputs.connected =
        BaseStatusSignal.isAllGood(velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp);
    inputs.velocityRps = velocity.getValue().in(RotationsPerSecond);
    inputs.appliedVoltage = appliedVoltage.getValue().in(Volts);
    inputs.supplyCurrentAmps = supplyCurrent.getValue().in(Amps);
    inputs.torqueCurrentAmps = torqueCurrent.getValue().in(Amps);
    inputs.tempCelsius = temp.getValue().in(Celsius);
  }

  @Override
  public void applyOutputs(KickerIOOutputs outputs) {
    talon.setControl(voltageRequest.withOutput(outputs.appliedVolts));
  }
}
