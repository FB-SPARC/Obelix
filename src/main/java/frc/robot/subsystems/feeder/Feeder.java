// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.subsystems.feeder.FeederIO.FeederIOOutputs;
import frc.robot.subsystems.feeder.FeederIO.FeederOutputMode;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Feeder subsystem that controls roller motors for transporting game pieces to the shooter.
 *
 * <p>Follows the FullSubsystem pattern: goals are stored in {@link #outputs} during commands and
 * applied atomically to the IO layer in {@link #periodicAfterScheduler()}.
 */
public class Feeder extends FullSubsystem {

  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private final FeederIOOutputs outputs = new FeederIOOutputs();

  // ── Tunable PID/FF gains ──────────────────────────────────────────────────
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Feeder/kP", FeederConstants.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Feeder/kI", FeederConstants.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Feeder/kD", FeederConstants.kD);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Feeder/kS", FeederConstants.kS);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Feeder/kV", FeederConstants.kV);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Feeder/kA", FeederConstants.kA);

  private final Alert leaderDisconnectedAlert =
      new Alert("Feeder leader motor disconnected!", AlertType.kError);
  private final Alert followerDisconnectedAlert =
      new Alert("Feeder follower motor disconnected!", AlertType.kError);

  /** Creates a new Feeder. */
  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);

    leaderDisconnectedAlert.set(Robot.showHardwareAlerts() && !inputs.leaderMotorConnected);
    followerDisconnectedAlert.set(Robot.showHardwareAlerts() && !inputs.followerMotorConnected);

    Robot.batteryLogger.reportCurrentUsage(
        "Feeder", false, inputs.leaderMotorSupplyCurrent + inputs.followerMotorSupplyCurrent);

    if (DriverStation.isDisabled()) {
      outputs.mode = FeederOutputMode.BRAKE;
    }

    if (kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get());
    }

    LoggedTracer.record("Feeder");
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Feeder/OutputMode", outputs.mode.toString());
    Logger.recordOutput("Feeder/GoalRPM", outputs.velocityRPM);
    Logger.recordOutput("Feeder/AtSetpoint", isAtSetpoint());
    io.applyOutputs(outputs);
  }

  // --- RPM API ---

  /**
   * Commands the feeder to a desired RPM.
   *
   * @param rpm the target roller RPM.
   */
  public void setFeederRPM(double rpm) {
    outputs.mode = FeederOutputMode.VELOCITY;
    outputs.velocityRPM = rpm;
  }

  /**
   * Returns the current leader roller RPM.
   *
   * @return leader roller RPM.
   */
  public double getFeederRPM() {
    return inputs.leaderMotorVelocityRPM;
  }

  /**
   * Returns whether the feeder is at the commanded RPM setpoint.
   *
   * @return true if the feeder RPM is within tolerance.
   */
  public boolean isAtSetpoint() {
    return Math.abs(inputs.leaderMotorVelocityRPM - outputs.velocityRPM)
        <= FeederConstants.kTolerance;
  }

  // --- Voltage API ---

  /**
   * Sets the feeder motor voltage directly (open-loop).
   *
   * @param voltage voltage from -12 to 12.
   */
  public void setVoltage(double voltage) {
    outputs.mode = FeederOutputMode.VOLTAGE;
    outputs.volts = voltage;
  }

  /** Stops the feeder motors (brake). */
  public void stop() {
    outputs.mode = FeederOutputMode.BRAKE;
  }

  /** Set brake mode (true) or coast mode (false). */
  public void setBrakeMode(boolean brake) {
    outputs.brakeMode = brake;
  }
}
