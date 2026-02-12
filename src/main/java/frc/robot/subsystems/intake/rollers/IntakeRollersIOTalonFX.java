// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.util.PhoenixUtil;

public class IntakeRollersIOTalonFX implements IntakeRollersIO {
  // TODO: Move CAN IDs into constants.
  // TODO: Change to single motor when hardware is updated.
  private static final int leaderId = 15;
  private static final int followerId = 16;

  private final TalonFX talon;
  private final TalonFX talonFollower;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  private final VoltageOut voltageRequest = new VoltageOut(0);

  public IntakeRollersIOTalonFX() {
    talon = new TalonFX(leaderId);
    talonFollower = new TalonFX(followerId);

    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;

    var followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> talonFollower.getConfigurator().apply(followerConfig));
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            talonFollower.setControl(
                new Follower(talon.getDeviceID(), MotorAlignmentValue.Aligned)));

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
    PhoenixUtil.tryUntilOk(5, () -> talonFollower.optimizeBusUtilization());
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {
    talon.setNeutralMode(enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    talonFollower.setNeutralMode(enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void updateInputs(IntakeRollersIOInputs inputs) {
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
  public void applyOutputs(IntakeRollersIOOutputs outputs) {
    talon.setControl(voltageRequest.withOutput(outputs.appliedVolts));
  }
}
