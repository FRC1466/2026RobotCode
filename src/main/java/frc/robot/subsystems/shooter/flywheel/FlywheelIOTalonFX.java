// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.util.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {
  // TODO: Move CAN IDs into constants.
  private static final int leaderId = 18;
  private static final int followerId = 19;

  private final TalonFX talon;
  private final TalonFX talonFollower;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  private final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0).withSlot(1);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  private double lastKp = 0.0;
  private double lastKd = 0.0;
  private double lastKs = 0.0;
  private double lastKv = 0.0;

  public FlywheelIOTalonFX() {
    talon = new TalonFX(leaderId);
    talonFollower = new TalonFX(followerId);

    var config = new TalonFXConfiguration();
    config.Slot1.kS = 0.24;
    config.Slot1.kP = 0.3;
    config.Slot1.kV = 0.1225;
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
    torqueCurrent = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp));
    PhoenixUtil.tryUntilOk(5, () -> talon.optimizeBusUtilization());
    PhoenixUtil.tryUntilOk(5, () -> talonFollower.optimizeBusUtilization());
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {
    talon.setNeutralMode(enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    talonFollower.setNeutralMode(enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
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
  public void applyOutputs(FlywheelIOOutputs outputs) {
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

    if (outputs.controlMode == FlywheelIOOutputs.ControlMode.VOLTAGE) {
      talon.setControl(voltageRequest.withOutput(outputs.appliedVolts));
    } else if (outputs.controlMode == FlywheelIOOutputs.ControlMode.DUTY_CYCLE_BANG_BANG) {
      double currentSpeed = velocity.getValue().in(RotationsPerSecond);
      if (currentSpeed < outputs.velocityRps) {
        talon.setControl(dutyCycleRequest.withOutput(1.0));
      } else {
        talon.setControl(dutyCycleRequest.withOutput(0.0));
      }
    } else {
      talon.setControl(
          request
              .withVelocity(RotationsPerSecond.of(outputs.velocityRps))
              .withFeedForward(outputs.feedForward));
    }
  }
}
