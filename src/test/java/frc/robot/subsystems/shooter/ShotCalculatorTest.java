// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import org.junit.jupiter.api.Test;

class ShotCalculatorTest {
  @Test
  void percentageOffsetScalesWithCalculatedFlywheelSpeed()
      throws NoSuchMethodException, InvocationTargetException, IllegalAccessException {
    ShotCalculator shotCalculator = new ShotCalculator();
    Method applyFlywheelSpeedOffsets =
        ShotCalculator.class.getDeclaredMethod("applyFlywheelSpeedOffsets", double.class);
    applyFlywheelSpeedOffsets.setAccessible(true);

    shotCalculator.incrementFlywheelSpeedOffsetPercent(0.05);

    assertEquals(42.0, (double) applyFlywheelSpeedOffsets.invoke(shotCalculator, 40.0), 1e-9);
    assertEquals(31.5, (double) applyFlywheelSpeedOffsets.invoke(shotCalculator, 30.0), 1e-9);
  }

  @Test
  void absoluteAndPercentageOffsetsAreBothApplied()
      throws NoSuchMethodException, InvocationTargetException, IllegalAccessException {
    ShotCalculator shotCalculator = new ShotCalculator();
    Method applyFlywheelSpeedOffsets =
        ShotCalculator.class.getDeclaredMethod("applyFlywheelSpeedOffsets", double.class);
    applyFlywheelSpeedOffsets.setAccessible(true);

    shotCalculator.incrementFlywheelSpeedOffsetPercent(0.05);
    shotCalculator.incrementFlywheelSpeedOffset(2.0);

    assertEquals(33.5, (double) applyFlywheelSpeedOffsets.invoke(shotCalculator, 30.0), 1e-9);
  }
}
