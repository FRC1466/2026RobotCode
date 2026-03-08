// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;

public class IntakePivotIOSlamTalonFX implements IntakePivotIO {
  private static final int motorId = 20;

  private static final LoggedTunableNumber deployVolts =
      new LoggedTunableNumber("IntakePivotSlam/DeployVolts", 2.0);
  private static final LoggedTunableNumber retractVolts =
      new LoggedTunableNumber("IntakePivotSlam/RetractVolts", -2.5);
  private static final LoggedTunableNumber deployDurationSec =
      new LoggedTunableNumber("IntakePivotSlam/DeployDurationSec", 0.75);
  private static final LoggedTunableNumber retractDurationSec =
      new LoggedTunableNumber("IntakePivotSlam/RetractDurationSec", 0.5);
  private static final LoggedTunableNumber brakeWindowScale =
      new LoggedTunableNumber("IntakePivotSlam/BrakeWindowScale", 0.2);
  private static final LoggedTunableNumber holdPulseIntervalSec =
      new LoggedTunableNumber("IntakePivotSlam/HoldPulseIntervalSec", 0.3);
  private static final LoggedTunableNumber holdPulseDurationSec =
      new LoggedTunableNumber("IntakePivotSlam/HoldPulseDurationSec", 0.1);
  private static final LoggedTunableNumber holdPulseDeployVolts =
      new LoggedTunableNumber("IntakePivotSlam/HoldPulseDeployVolts", 0.8);
  private static final LoggedTunableNumber holdPulseRetractVolts =
      new LoggedTunableNumber("IntakePivotSlam/HoldPulseRetractVolts", -0.8);

  private static final double deployThresholdRotations = 0.1;
  private static final double goalChangeThresholdRotations = 0.01;

  private final TalonFX talon;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  private final VoltageOut voltageRequest = new VoltageOut(0);

  private enum SlamState {
    IDLE,
    DEPLOYING,
    RETRACTING
  }

  private SlamState state = SlamState.IDLE;
  private final Timer slamTimer = new Timer();
  private final Timer holdPulseTimer = new Timer();
  private double lastGoalRotations = 0.0;
  private boolean lastGoalWasDeploy = false;

  public IntakePivotIOSlamTalonFX() {
    talon = new TalonFX(motorId, TunerConstants.kCANBus);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
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

    holdPulseTimer.start();
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp);
    inputs.motorConnected =
        BaseStatusSignal.isAllGood(
            position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp);
    inputs.positionRotations = position.getValue().in(Rotations);
    inputs.velocityRotationsPerSec = velocity.getValue().in(RotationsPerSecond);
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
  public void applyOutputs(IntakePivotIOOutputs outputs) {
    if (outputs.mode == IntakePivotIOOutputMode.OPEN_LOOP) {
      state = SlamState.IDLE;
      holdPulseTimer.restart();
      talon.setControl(voltageRequest.withOutput(outputs.volts));
      return;
    }

    double goalRotations = outputs.positionRotations;
    boolean goalChanged =
        Math.abs(goalRotations - lastGoalRotations) > goalChangeThresholdRotations;
    boolean goalIsDeploy = goalRotations >= deployThresholdRotations;
    lastGoalRotations = goalRotations;

    if (goalChanged) {
      lastGoalWasDeploy = goalIsDeploy;
      state = goalIsDeploy ? SlamState.DEPLOYING : SlamState.RETRACTING;
      slamTimer.restart();
      holdPulseTimer.restart();
    }

    switch (state) {
      case DEPLOYING -> {
        double elapsedSec = slamTimer.get();
        double durationSec = deployDurationSec.get();
        double targetVolts = deployVolts.get();
        double brakeScale = brakeWindowScale.get();

        if (elapsedSec < durationSec) {
          talon.setControl(
              voltageRequest.withOutput(
                  calculateBrakedVoltage(targetVolts, elapsedSec, durationSec, brakeScale)));
        } else {
          talon.setControl(voltageRequest.withOutput(0.0));
          state = SlamState.IDLE;
          holdPulseTimer.restart();
        }
      }
      case RETRACTING -> {
        double elapsedSec = slamTimer.get();
        double durationSec = retractDurationSec.get();
        double targetVolts = retractVolts.get();
        double brakeScale = brakeWindowScale.get();

        if (elapsedSec < durationSec) {
          talon.setControl(
              voltageRequest.withOutput(
                  calculateBrakedVoltage(targetVolts, elapsedSec, durationSec, brakeScale)));
        } else {
          talon.setControl(voltageRequest.withOutput(0.0));
          state = SlamState.IDLE;
          holdPulseTimer.restart();
        }
      }
      case IDLE -> {
        double intervalSec = holdPulseIntervalSec.get();
        double pulseDurationSec = holdPulseDurationSec.get();
        boolean pulsing =
            holdPulseTimer.get() >= intervalSec
                && holdPulseTimer.get() < intervalSec + pulseDurationSec;

        if (pulsing) {
          double holdVolts =
              lastGoalWasDeploy ? holdPulseDeployVolts.get() : holdPulseRetractVolts.get();
          talon.setControl(voltageRequest.withOutput(holdVolts));
        } else {
          talon.setControl(voltageRequest.withOutput(0.0));
          if (holdPulseTimer.get() >= intervalSec + pulseDurationSec) {
            holdPulseTimer.restart();
          }
        }
      }
    }
  }

  static double calculateBrakedVoltage(
      double targetVolts, double elapsedSec, double durationSec, double brakeWindowScale) {
    if (brakeWindowScale <= 0.0) {
      return targetVolts;
    }
    double brakeWindowSec = durationSec * MathUtil.clamp(brakeWindowScale, 0.0, 1.0);
    if (brakeWindowSec <= 0.0) {
      return targetVolts;
    }

    double brakeStartSec = durationSec - brakeWindowSec;
    if (elapsedSec <= brakeStartSec) {
      return targetVolts;
    }

    double brakeProgress =
        MathUtil.clamp((elapsedSec - brakeStartSec) / brakeWindowSec, 0.0, 1.0);
    return targetVolts * (1.0 - brakeProgress);
  }
}
