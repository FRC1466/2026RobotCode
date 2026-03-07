// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ShotCalculatorTest {
  @Test
  void percentageOffsetScalesWithCalculatedFlywheelSpeed() {
    ShotCalculator shotCalculator = new ShotCalculator();

    shotCalculator.incrementFlywheelSpeedOffsetPercent(0.05);

    assertEquals(42.0, shotCalculator.applyFlywheelSpeedOffsets(40.0), 1e-9);
    assertEquals(31.5, shotCalculator.applyFlywheelSpeedOffsets(30.0), 1e-9);
  }

  @Test
  void absoluteAndPercentageOffsetsAreBothApplied() {
    ShotCalculator shotCalculator = new ShotCalculator();

    shotCalculator.incrementFlywheelSpeedOffsetPercent(0.05);
    shotCalculator.incrementFlywheelSpeedOffset(2.0);

    assertEquals(33.5, shotCalculator.applyFlywheelSpeedOffsets(30.0), 1e-9);
  }
}
