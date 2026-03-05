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
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;

/**
 * Intake pivot IO that "slams" the pivot using timed open-loop voltage instead of closed-loop
 * position control. Deploy runs voltage for a short burst (0.25 s), retract runs voltage for a
 * longer duration (1.0 s), then the motor stops.
 */
public class IntakePivotIOSlamTalonFX implements IntakePivotIO {
  private static final int motorId = 20;

  private static final LoggedTunableNumber deployVolts =
      new LoggedTunableNumber("IntakePivotSlam/DeployVolts", 1.0);
  private static final LoggedTunableNumber retractVolts =
      new LoggedTunableNumber("IntakePivotSlam/RetractVolts", -2.5);
  private static final LoggedTunableNumber deployDurationSec =
      new LoggedTunableNumber("IntakePivotSlam/DeployDurationSec", 0.75);
  private static final LoggedTunableNumber retractDurationSec =
      new LoggedTunableNumber("IntakePivotSlam/RetractDurationSec", .5);

  // Threshold in rotations to distinguish deploy vs retract goals.
  // Deploy goal is typically ~105 deg (0.292 rot), stow is ~0.1 deg (~0.0003 rot).
  private static final double deployThresholdRotations = 0.1;

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
  private double lastGoalRotations = 0.0;

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
      // Direct open-loop passthrough (e.g. from runVolts command)
      state = SlamState.IDLE;
      talon.setControl(voltageRequest.withOutput(outputs.volts));
      return;
    }

    // Closed-loop mode: detect goal changes and convert to timed voltage slam
    double goalRotations = outputs.positionRotations;
    boolean goalChanged = Math.abs(goalRotations - lastGoalRotations) > 0.01;
    lastGoalRotations = goalRotations;

    if (goalChanged) {
      boolean deploying = goalRotations >= deployThresholdRotations;
      state = deploying ? SlamState.DEPLOYING : SlamState.RETRACTING;
      slamTimer.restart();
    }

    switch (state) {
      case DEPLOYING -> {
        if (slamTimer.get() < deployDurationSec.get()) {
          talon.setControl(voltageRequest.withOutput(deployVolts.get()));
        } else {
          talon.setControl(voltageRequest.withOutput(0.0));
          state = SlamState.IDLE;
        }
      }
      case RETRACTING -> {
        if (slamTimer.get() < retractDurationSec.get()) {
          talon.setControl(voltageRequest.withOutput(retractVolts.get()));
        } else {
          talon.setControl(voltageRequest.withOutput(0.0));
          state = SlamState.IDLE;
        }
      }
      case IDLE -> {
        talon.setControl(voltageRequest.withOutput(0.0));
      }
    }
  }
}
