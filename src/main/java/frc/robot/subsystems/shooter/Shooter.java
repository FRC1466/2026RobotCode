// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Shooter extends FullSubsystem {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIOOutputs outputs = new ShooterIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;

  @Setter private double volts = 0.0;
  private static final LoggedTunableNumber rateLimiter =
      new LoggedTunableNumber("Shooter/SlewRateLimiter", 100);
  SlewRateLimiter slewRateLimiter = new SlewRateLimiter(rateLimiter.get());

  public Shooter(ShooterIO io) {
    this.io = io;

    disconnected = new Alert("Shooter motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if (rateLimiter.hasChanged(hashCode())) {
      slewRateLimiter = new SlewRateLimiter(rateLimiter.get());
    }
    outputs.kP = ShooterConstants.kp.get();
    outputs.kD = ShooterConstants.kd.get();

    if (outputs.coast) {
      slewRateLimiter.reset(inputs.velocityRadsPerSec);
    }

    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRadsPerSec) {
    outputs.coast = false;
    outputs.velocityRadsPerSec = slewRateLimiter.calculate(velocityRadsPerSec);
    outputs.feedForward = ShooterConstants.ks * Math.signum(velocityRadsPerSec);

    // Log shooter setpoint
    Logger.recordOutput("Shooter/Setpoint", outputs.velocityRadsPerSec);
  }

  /** Stops the shooter. */
  public void stop() {
    outputs.velocityRadsPerSec = 0.0;
    outputs.coast = true;
  }

  /** Returns the current velocity in RPM. */
  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }
}
