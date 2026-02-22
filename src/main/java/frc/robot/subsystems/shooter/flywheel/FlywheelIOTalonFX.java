// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.util.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {
  // TODO: Move CAN IDs into constants.
  private static final int leaderId = 40;
  private static final int followerId = 41;

  private final TalonFX talon;
  private final TalonFX talonFollower;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> supplyCurrentFollower;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Temperature> tempFollower;

  private final MotionMagicVelocityVoltage voltageRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  private final MotionMagicVelocityTorqueCurrentFOC torqueCurrentRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(1);
  private final com.ctre.phoenix6.controls.VoltageOut voltageOutRequest =
      new com.ctre.phoenix6.controls.VoltageOut(0);

  private double lastVoltageKP = 0.0;
  private double lastVoltageKD = 0.0;
  private double lastVoltageKS = 0.0;
  private double lastVoltageKV = 0.0;
  private double lastTorqueCurrentKP = 0.0;
  private double lastTorqueCurrentKD = 0.0;
  private double lastTorqueCurrentKS = 0.0;
  private double lastTorqueCurrentKV = 0.0;

  public FlywheelIOTalonFX() {
    talon = new TalonFX(leaderId);
    talonFollower = new TalonFX(followerId);

    var config = new TalonFXConfiguration();
    config.MotionMagic.MotionMagicAcceleration = 500.0;
    config.MotionMagic.MotionMagicJerk = 0.0;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = 1.0;

    var followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> talonFollower.getConfigurator().apply(followerConfig));
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            talonFollower.setControl(
                new Follower(talon.getDeviceID(), MotorAlignmentValue.Opposed)));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    supplyCurrentFollower = talonFollower.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();
    tempFollower = talonFollower.getDeviceTemp();

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                supplyCurrentFollower,
                torqueCurrent,
                temp,
                tempFollower));
    PhoenixUtil.tryUntilOk(5, () -> talon.optimizeBusUtilization());
    PhoenixUtil.tryUntilOk(5, () -> talonFollower.optimizeBusUtilization());
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp);
    BaseStatusSignal.refreshAll(supplyCurrentFollower, tempFollower);
    inputs.connectedMaster =
        BaseStatusSignal.isAllGood(
            position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp);
    inputs.connectedFollower = BaseStatusSignal.isAllGood(supplyCurrentFollower, tempFollower);
    inputs.positionRotations = position.getValue().in(Rotations);
    inputs.velocityRotationsPerSec = velocity.getValue().in(RotationsPerSecond);
    inputs.appliedVoltage = appliedVoltage.getValue().in(Volts);
    inputs.supplyCurrentMasterAmps = supplyCurrent.getValue().in(Amps);
    inputs.supplyCurrentFollowerAmps = supplyCurrentFollower.getValue().in(Amps);
    inputs.torqueCurrentAmps = torqueCurrent.getValue().in(Amps);
    inputs.tempMasterCelsius = temp.getValue().in(Celsius);
    inputs.tempFollowerCelsius = tempFollower.getValue().in(Celsius);
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    if (outputs.voltageKP != lastVoltageKP
        || outputs.voltageKD != lastVoltageKD
        || outputs.voltageKS != lastVoltageKS
        || outputs.voltageKV != lastVoltageKV) {
      var slot0 = new Slot0Configs();
      slot0.kP = outputs.voltageKP;
      slot0.kD = outputs.voltageKD;
      slot0.kS = outputs.voltageKS;
      slot0.kV = outputs.voltageKV;
      PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(slot0));
      lastVoltageKP = outputs.voltageKP;
      lastVoltageKD = outputs.voltageKD;
      lastVoltageKS = outputs.voltageKS;
      lastVoltageKV = outputs.voltageKV;
    }

    if (outputs.torqueCurrentKP != lastTorqueCurrentKP
        || outputs.torqueCurrentKD != lastTorqueCurrentKD
        || outputs.torqueCurrentKS != lastTorqueCurrentKS
        || outputs.torqueCurrentKV != lastTorqueCurrentKV) {
      var slot1 = new Slot1Configs();
      slot1.kP = outputs.torqueCurrentKP;
      slot1.kD = outputs.torqueCurrentKD;
      slot1.kS = outputs.torqueCurrentKS;
      slot1.kV = outputs.torqueCurrentKV;
      PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(slot1));
      lastTorqueCurrentKP = outputs.torqueCurrentKP;
      lastTorqueCurrentKD = outputs.torqueCurrentKD;
      lastTorqueCurrentKS = outputs.torqueCurrentKS;
      lastTorqueCurrentKV = outputs.torqueCurrentKV;
    }

    switch (outputs.mode) {
      case VOLTAGE -> talon.setControl(voltageOutRequest.withOutput(outputs.feedforward));
      case VELOCITY_VOLTAGE ->
          talon.setControl(
              voltageRequest
                  .withVelocity(RotationsPerSecond.of(outputs.velocityRotationsPerSec))
                  .withFeedForward(outputs.feedforward));
      case VELOCITY_TORQUE_CURRENT ->
          talon.setControl(
              torqueCurrentRequest
                  .withVelocity(RotationsPerSecond.of(outputs.velocityRotationsPerSec))
                  .withFeedForward(outputs.feedforward));
      default -> talon.setControl(voltageOutRequest.withOutput(0));
    }
  }
}
