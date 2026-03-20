// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/** Utility class for logging subsystem motor currents to AdvantageKit. */
public class BatteryTracer {
  private BatteryTracer() {}

  // Stores accumulated current per subsystem for the current cycle
  private static final Map<String, Double> currentMap = new HashMap<>();

  /** Reset all tracked subsystem currents. Should be called at the start of each cycle. */
  public static void reset() {
    currentMap.clear();
  }

  /**
   * Add current draw for a subsystem. Call this from each subsystem as you update motor currents.
   *
   * @param subsystemName Name of the subsystem (e.g. "Drive", "Flywheel")
   * @param current Current draw in amps to add for this subsystem
   */
  public static void addCurrent(String subsystemName, double current) {
    currentMap.merge(subsystemName, current, Double::sum);
  }

  /**
   * Publish the accumulated current for a subsystem to AdvantageKit log, then reset its
   * accumulator. Should be called at the end of the subsystem's periodic.
   *
   * @param subsystemName Name of the subsystem
   */
  public static void publish(String subsystemName) {
    double totalCurrent = currentMap.getOrDefault(subsystemName, 0.0);
    Logger.recordOutput("BatteryTracer/" + subsystemName + "Current", totalCurrent);
    currentMap.remove(subsystemName);
  }
}
