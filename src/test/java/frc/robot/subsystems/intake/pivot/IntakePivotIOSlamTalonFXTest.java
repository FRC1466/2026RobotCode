// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.intake.pivot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class IntakePivotIOSlamTalonFXTest {
  @Test
  void calculateBrakedVoltageStaysAtTargetUntilBrakeWindow() {
    assertEquals(2.0, IntakePivotIOSlamTalonFX.calculateBrakedVoltage(2.0, 0.0, 0.75, 0.2), 1e-9);
    assertEquals(2.0, IntakePivotIOSlamTalonFX.calculateBrakedVoltage(2.0, 0.59, 0.75, 0.2), 1e-9);
  }

  @Test
  void calculateBrakedVoltageRampsDownToZeroAtEnd() {
    assertEquals(1.0, IntakePivotIOSlamTalonFX.calculateBrakedVoltage(2.0, 0.675, 0.75, 0.2), 1e-9);
    assertEquals(0.0, IntakePivotIOSlamTalonFX.calculateBrakedVoltage(2.0, 0.75, 0.75, 0.2), 1e-9);
  }

  @Test
  void calculateBrakedVoltagePreservesRetractDirection() {
    assertEquals(
        -1.25, IntakePivotIOSlamTalonFX.calculateBrakedVoltage(-2.5, 0.45, 0.5, 0.2), 1e-9);
  }

  @Test
  void calculateBrakedVoltageBypassesBrakingWhenDisabled() {
    assertEquals(2.0, IntakePivotIOSlamTalonFX.calculateBrakedVoltage(2.0, 0.74, 0.75, 0.0), 1e-9);
    assertEquals(2.0, IntakePivotIOSlamTalonFX.calculateBrakedVoltage(2.0, 0.74, 0.75, -1.0), 1e-9);
  }
}
