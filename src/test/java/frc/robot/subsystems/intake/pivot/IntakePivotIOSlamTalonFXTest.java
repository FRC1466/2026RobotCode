// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.intake.pivot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class IntakePivotIOSlamTalonFXTest {
  @Test
  void calculateRampedVoltageStartsAtZeroAndReachesTarget() {
    assertEquals(0.0, IntakePivotIOSlamTalonFX.calculateRampedVoltage(2.0, 0.0, 0.2), 1e-9);
    assertEquals(1.0, IntakePivotIOSlamTalonFX.calculateRampedVoltage(2.0, 0.1, 0.2), 1e-9);
    assertEquals(2.0, IntakePivotIOSlamTalonFX.calculateRampedVoltage(2.0, 0.2, 0.2), 1e-9);
    assertEquals(2.0, IntakePivotIOSlamTalonFX.calculateRampedVoltage(2.0, 0.5, 0.2), 1e-9);
  }

  @Test
  void calculateRampedVoltagePreservesRetractDirection() {
    assertEquals(
        -1.25, IntakePivotIOSlamTalonFX.calculateRampedVoltage(-2.5, 0.05, 0.1), 1e-9);
  }

  @Test
  void calculateRampedVoltageBypassesRampWhenDisabled() {
    assertEquals(2.0, IntakePivotIOSlamTalonFX.calculateRampedVoltage(2.0, 0.0, 0.0), 1e-9);
    assertEquals(2.0, IntakePivotIOSlamTalonFX.calculateRampedVoltage(2.0, 0.0, -1.0), 1e-9);
  }
}
