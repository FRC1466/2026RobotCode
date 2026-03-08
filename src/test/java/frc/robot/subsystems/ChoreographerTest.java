// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ChoreographerTest {
  @Test
  void doneShootingRequiresNewShotAfterScoreGoalStarts() {
    assertFalse(Choreographer.isDoneShooting(5.0, 5.0, 0.5, 0.25));
    assertTrue(Choreographer.isDoneShooting(5.0, 5.2, 0.3, 0.25));
  }

  @Test
  void doneShootingWaitsForShotToSettle() {
    assertFalse(Choreographer.isDoneShooting(5.0, 5.2, 0.1, 0.25));
  }
}
