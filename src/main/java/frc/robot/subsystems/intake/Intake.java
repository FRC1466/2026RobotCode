// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFX intakeMotor1 = new TalonFX(15);
  private TalonFX intakeMotor2 = new TalonFX(16);

  private double appliedVolts = 0.0;
  private double dutyCycle = 0.0;

  private boolean useVoltage = true;

  private VoltageOut voltageControl = new VoltageOut(appliedVolts);
  private DutyCycleOut dutyCycleControl = new DutyCycleOut(dutyCycle);

  public Intake() {
    MotorOutputConfigs currentConfigs = new MotorOutputConfigs();

    // The left motor is CCW+
    currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
    intakeMotor1.getConfigurator().apply(currentConfigs);

    intakeMotor2.setControl(new Follower(intakeMotor1.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  @Override
  public void periodic() {
    if (useVoltage) {
      intakeMotor1.setControl(voltageControl.withOutput(appliedVolts));
    } else {
      intakeMotor1.setControl(dutyCycleControl.withOutput(dutyCycle));
    }
  }

  public void setVoltage(double volts) {
    useVoltage = true;
    appliedVolts = volts;
  }

  public void setDutyCycle(double duty) {
    useVoltage = false;
    dutyCycle = duty;
  }

  public void stop() {
    setVoltage(0.0);
  }
}
