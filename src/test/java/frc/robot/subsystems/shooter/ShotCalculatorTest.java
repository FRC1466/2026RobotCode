// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.concurrent.atomic.AtomicReference;
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

  @Test
  void timeSinceLastShotTracksRecordedShotTime() {
    AtomicReference<Double> timestamp = new AtomicReference<>(5.0);
    ShotCalculator shotCalculator = new ShotCalculator(timestamp::get);

    assertEquals(Double.POSITIVE_INFINITY, shotCalculator.getTimeSinceLastShotSeconds());

    shotCalculator.recordShot();
    assertEquals(0.0, shotCalculator.getTimeSinceLastShotSeconds(), 1e-9);

    timestamp.set(5.25);
    assertEquals(0.25, shotCalculator.getTimeSinceLastShotSeconds(), 1e-9);

    timestamp.set(6.0);
    shotCalculator.recordShot();
    timestamp.set(6.4);
    assertEquals(0.4, shotCalculator.getTimeSinceLastShotSeconds(), 1e-9);
  }
}
