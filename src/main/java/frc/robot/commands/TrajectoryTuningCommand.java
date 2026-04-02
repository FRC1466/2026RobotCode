// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

/**
 * Relay (bang-bang) auto-tuner for the Choreo trajectory-following PID controllers.
 *
 * <p>Uses the Åström-Hägglund relay method to find the ultimate gain and period of the translation
 * and heading position loops. The robot oscillates around its starting pose with very small
 * amplitude (typically ±0.05 m and ±0.05 rad) — no large field space required.
 *
 * <p>After running, check AdvantageKit under {@code TrajectoryTuning/Results/} for the recommended
 * kP and kD values to plug into {@code TRANSLATION_CONTROLLER_CONSTANTS} and {@code
 * HEADING_CONTROLLER_CONSTANTS} in {@code Drive.java}.
 */
public class TrajectoryTuningCommand extends Command {

  // How hard the relay pushes (m/s for translation, rad/s for heading).
  // Smaller = tighter oscillation but noisier signal; larger = wider oscillation but cleaner.
  private static final double TRANSLATION_RELAY_AMP = 0.6; // m/s
  private static final double HEADING_RELAY_AMP = 2.0; // rad/s

  // How long to run each relay test.
  private static final double RELAY_DURATION = 8.0; // seconds

  // Minimum oscillations before accepting the result (filters out transients).
  private static final int MIN_OSCILLATIONS = 5;

  private enum Phase {
    PREFLIGHT,
    TRANSLATION_RELAY,
    HEADING_RELAY,
    COMPLETE
  }

  private final Drive drive;
  private Phase phase = Phase.PREFLIGHT;
  private final Timer phaseTimer = new Timer();

  private Pose2d startPose;

  // Relay state
  private boolean relayPositive = true;
  private double lastErrorSign = 0;

  // Zero-crossing tracking (each crossing = half oscillation)
  private final ArrayList<Double> crossingTimes = new ArrayList<>();
  private final ArrayList<Double> peakAmplitudes = new ArrayList<>();
  private double currentPeakAbs = 0;

  // Results
  private double translationKp = 5.0;
  private double translationKd = 0.05;
  private double headingKp = 10.0;
  private double headingKd = 0.5;

  public TrajectoryTuningCommand(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    startPose = drive.getPose();
    phase = Phase.PREFLIGHT;
    phaseTimer.restart();
    Logger.recordOutput("TrajectoryTuning/Phase", phase.name());
    Logger.recordOutput("TrajectoryTuning/Status", "Starting — keep robot in open space");
  }

  @Override
  public void execute() {
    Logger.recordOutput("TrajectoryTuning/Phase", phase.name());

    switch (phase) {
      case PREFLIGHT -> {
        drive.stop();
        if (phaseTimer.hasElapsed(0.5)) {
          startPose = drive.getPose();
          resetRelayState();
          advanceTo(Phase.TRANSLATION_RELAY);
        }
      }
      case TRANSLATION_RELAY -> {
        double error = drive.getPose().getX() - startPose.getX();
        runRelay(error, TRANSLATION_RELAY_AMP, true);

        if (phaseTimer.hasElapsed(RELAY_DURATION)) {
          RelayResult result = computeGains(crossingTimes, peakAmplitudes, TRANSLATION_RELAY_AMP);
          if (result != null) {
            translationKp = result.kP;
            translationKd = result.kD;
          }
          Logger.recordOutput("TrajectoryTuning/Results/TranslationKP", translationKp);
          Logger.recordOutput("TrajectoryTuning/Results/TranslationKD", translationKd);
          resetRelayState();
          advanceTo(Phase.HEADING_RELAY);
        }
      }
      case HEADING_RELAY -> {
        double error =
            MathUtil.angleModulus(
                drive.getPose().getRotation().getRadians() - startPose.getRotation().getRadians());
        runRelay(error, HEADING_RELAY_AMP, false);

        if (phaseTimer.hasElapsed(RELAY_DURATION)) {
          RelayResult result = computeGains(crossingTimes, peakAmplitudes, HEADING_RELAY_AMP);
          if (result != null) {
            headingKp = result.kP;
            headingKd = result.kD;
          }
          Logger.recordOutput("TrajectoryTuning/Results/HeadingKP", headingKp);
          Logger.recordOutput("TrajectoryTuning/Results/HeadingKD", headingKd);
          advanceTo(Phase.COMPLETE);
        }
      }
      case COMPLETE -> {
        drive.stop();
        Logger.recordOutput("TrajectoryTuning/Status", "Done — check TrajectoryTuning/Results/");
      }
    }
  }

  /**
   * Applies a relay (bang-bang) corrective velocity toward the start pose to induce a limit cycle.
   *
   * @param error current error (positive = robot is ahead of target)
   * @param relayAmp relay output magnitude
   * @param isTranslation true = push along start heading (translation), false = push rotationally
   */
  private void runRelay(double error, double relayAmp, boolean isTranslation) {
    // Switch relay direction when error crosses zero
    double errorSign = Math.signum(error);
    if (lastErrorSign != 0 && errorSign != lastErrorSign && errorSign != 0) {
      crossingTimes.add(phaseTimer.get());
      peakAmplitudes.add(currentPeakAbs);
      currentPeakAbs = 0;
    }
    if (errorSign != 0) {
      lastErrorSign = errorSign;
    }
    currentPeakAbs = Math.max(currentPeakAbs, Math.abs(error));

    // Bang-bang: push against the error
    boolean shouldBePositive = error < 0; // push positive when error is negative
    if (errorSign != 0) {
      relayPositive = shouldBePositive;
    }

    double correction = relayPositive ? relayAmp : -relayAmp;

    ChassisSpeeds speeds;
    if (isTranslation) {
      double cos = startPose.getRotation().getCos();
      double sin = startPose.getRotation().getSin();
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              correction * cos, correction * sin, 0.0, drive.getPose().getRotation());
    } else {
      speeds = new ChassisSpeeds(0, 0, correction);
    }
    drive.runVelocity(speeds);

    Logger.recordOutput("TrajectoryTuning/Relay/Error", error);
    Logger.recordOutput("TrajectoryTuning/Relay/Output", correction);
    Logger.recordOutput("TrajectoryTuning/Relay/Crossings", crossingTimes.size());
  }

  /**
   * Computes PD gains from the relay test data using Ziegler-Nichols relay method.
   *
   * <p>Ku = 4 * relayAmp / (π * avg_amplitude) <br>
   * Tu = avg time between zero crossings * 2 <br>
   * kP = 0.5 * Ku (PD tuning), kD = Ku * Tu / 8
   */
  private RelayResult computeGains(
      ArrayList<Double> crossings, ArrayList<Double> amplitudes, double relayAmp) {

    if (crossings.size() < MIN_OSCILLATIONS || amplitudes.isEmpty()) {
      Logger.recordOutput(
          "TrajectoryTuning/Status",
          "Insufficient oscillations (" + crossings.size() + ") — check relay amplitude");
      return null;
    }

    // Average half-period from zero crossings
    double totalTime = crossings.get(crossings.size() - 1) - crossings.get(0);
    double halfPeriod = totalTime / (crossings.size() - 1);
    double Tu = halfPeriod * 2.0;

    // Average amplitude (discard first crossing which may be a transient)
    int startIdx = Math.min(2, amplitudes.size() - 1);
    double avgAmplitude = 0;
    int count = 0;
    for (int i = startIdx; i < amplitudes.size(); i++) {
      avgAmplitude += amplitudes.get(i);
      count++;
    }
    avgAmplitude = (count > 0) ? avgAmplitude / count : amplitudes.get(amplitudes.size() - 1);

    if (avgAmplitude < 1e-4) {
      Logger.recordOutput("TrajectoryTuning/Status", "Amplitude too small — increase relay amp");
      return null;
    }

    double Ku = (4.0 * relayAmp) / (Math.PI * avgAmplitude);

    // PD Ziegler-Nichols
    double kP = 0.5 * Ku;
    double kD = Ku * Tu / 8.0;

    Logger.recordOutput("TrajectoryTuning/Relay/Ku", Ku);
    Logger.recordOutput("TrajectoryTuning/Relay/Tu", Tu);
    Logger.recordOutput("TrajectoryTuning/Relay/AvgAmplitude", avgAmplitude);

    return new RelayResult(kP, kD);
  }

  private void resetRelayState() {
    relayPositive = true;
    lastErrorSign = 0;
    crossingTimes.clear();
    peakAmplitudes.clear();
    currentPeakAbs = 0;
  }

  private void advanceTo(Phase next) {
    phase = next;
    phaseTimer.restart();
    drive.stop();
    Logger.recordOutput("TrajectoryTuning/Phase", phase.name());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    if (interrupted) {
      Logger.recordOutput("TrajectoryTuning/Status", "Interrupted");
    }
  }

  @Override
  public boolean isFinished() {
    return phase == Phase.COMPLETE && phaseTimer.hasElapsed(0.2);
  }

  private record RelayResult(double kP, double kD) {}
}
