// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter.flywheel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class FlywheelTest {
  @Test
  void shotIsDetectedWhenCurrentSpikesAboveAverageAtSpeed() {
    assertTrue(Flywheel.shouldRecordShotFromCurrentSpike(18.0, 10.0, 30.0, 25.0, 5.0));
  }

  @Test
  void shotIsNotDetectedWhenVelocityIsTooLow() {
    assertFalse(Flywheel.shouldRecordShotFromCurrentSpike(18.0, 10.0, 20.0, 25.0, 5.0));
  }

  @Test
  void shotDetectionCurrentAverageWindowAlwaysUsesAtLeastOneSample() {
    assertEquals(1, Flywheel.getShotDetectionCurrentAverageWindowSamples(0.0));
    assertEquals(
        (int) Math.round(2.0 / Constants.loopPeriodSecs),
        Flywheel.getShotDetectionCurrentAverageWindowSamples(2.0));
  }
}
