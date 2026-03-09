// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class DriveCommandsTest {
  @Test
  void disabledZoneLockRejectsBumpAndTrenchRequests() {
    assertEquals(
        DriveCommands.DriveMode.NORMAL,
        DriveCommands.sanitizeRequestedDriveMode(DriveCommands.DriveMode.TRENCH_LOCK, false));
    assertEquals(
        DriveCommands.DriveMode.NORMAL,
        DriveCommands.sanitizeRequestedDriveMode(DriveCommands.DriveMode.BUMP_LOCK, false));
  }

  @Test
  void launchLockIgnoresZoneAlignmentWhenDisabled() {
    assertEquals(
        DriveCommands.DriveMode.LAUNCH_LOCK,
        DriveCommands.resolveEffectiveMode(
            DriveCommands.DriveMode.NORMAL, true, false, true, false));
    assertEquals(
        DriveCommands.DriveMode.LAUNCH_LOCK,
        DriveCommands.resolveEffectiveMode(
            DriveCommands.DriveMode.NORMAL, true, false, false, true));
  }

  @Test
  void enabledZoneLockStillPrefersTrenchThenBumpDuringLaunch() {
    assertEquals(
        DriveCommands.DriveMode.TRENCH_LOCK,
        DriveCommands.resolveEffectiveMode(
            DriveCommands.DriveMode.NORMAL, true, true, true, true));
    assertEquals(
        DriveCommands.DriveMode.BUMP_LOCK,
        DriveCommands.resolveEffectiveMode(
            DriveCommands.DriveMode.NORMAL, true, true, false, true));
  }
}
