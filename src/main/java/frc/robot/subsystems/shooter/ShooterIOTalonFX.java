// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX talon;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0).withSlot(1);

  private double lastKp = 0.3;
  private double lastKd = 0.0;
  private double lastKs = 0.19;
  private double lastKv = 0.11;
  private boolean lastCoast = true;

  public ShooterIOTalonFX() {
    talon = new TalonFX(14);

    var config = new TalonFXConfiguration();
    config.Slot1.kS = 0.19;
    config.Slot1.kP = 0.3;
    config.Slot1.kV = 0.11;
    config.MotionMagic.MotionMagicAcceleration = 500.0;
    config.MotionMagic.MotionMagicJerk = 0.0;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = 1.0;

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
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp);
    inputs.connected =
        BaseStatusSignal.isAllGood(
            position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp);
    inputs.positionRotations = position.getValue().in(Rotations);
    inputs.velocityRps = velocity.getValue().in(RotationsPerSecond);
    inputs.appliedVoltage = appliedVoltage.getValue().in(Volts);
    inputs.supplyCurrentAmps = supplyCurrent.getValue().in(Amps);
    inputs.torqueCurrentAmps = torqueCurrent.getValue().in(Amps);
    inputs.tempCelsius = temp.getValue().in(Celsius);
  }

  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    if (outputs.coast != lastCoast) {
      talon.setNeutralMode(outputs.coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
      lastCoast = outputs.coast;
    }

    if (outputs.kP != 0.0
        && (outputs.kP != lastKp
            || outputs.kD != lastKd
            || outputs.kS != lastKs
            || outputs.kV != lastKv)) {
      var slot1 = new Slot1Configs();
      slot1.kP = outputs.kP;
      slot1.kD = outputs.kD;
      slot1.kS = outputs.kS;
      slot1.kV = outputs.kV;
      PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(slot1));
      lastKp = outputs.kP;
      lastKd = outputs.kD;
      lastKs = outputs.kS;
      lastKv = outputs.kV;
    }

    talon.setControl(
        request
            .withVelocity(RotationsPerSecond.of(outputs.velocityRps))
            .withFeedForward(outputs.feedForward));
  }
}
