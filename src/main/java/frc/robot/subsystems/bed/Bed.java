// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.bed;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.subsystems.bed.BedIO.BedIOOutputs;
import frc.robot.subsystems.bed.BedIO.BedOutputMode;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Bed subsystem that controls a bed of rollers for transporting game pieces.
 *
 * <p>Follows the FullSubsystem pattern: goals are stored in {@link #outputs} during commands and
 * applied atomically to the IO layer in {@link #periodicAfterScheduler()}.
 */
public class Bed extends FullSubsystem {

  private final BedIO io;
  private final BedIOInputsAutoLogged inputs = new BedIOInputsAutoLogged();
  private final BedIOOutputs outputs = new BedIOOutputs();

  // ── Tunable PID/FF gains ──────────────────────────────────────────────────
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Bed/kP", BedConstants.kP);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Bed/kI", BedConstants.kI);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Bed/kD", BedConstants.kD);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Bed/kS", BedConstants.kS);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Bed/kV", BedConstants.kV);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Bed/kA", BedConstants.kA);

  private final Alert leaderDisconnectedAlert =
      new Alert("Bed leader motor disconnected!", AlertType.kError);
  private final Alert followerDisconnectedAlert =
      new Alert("Bed follower motor disconnected!", AlertType.kError);

  /** Creates a new Bed. */
  public Bed(BedIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Bed", inputs);

    leaderDisconnectedAlert.set(Robot.showHardwareAlerts() && !inputs.leaderMotorConnected);
    followerDisconnectedAlert.set(Robot.showHardwareAlerts() && !inputs.followerMotorConnected);

    Robot.batteryLogger.reportCurrentUsage(
        "Bed", false, inputs.leaderMotorSupplyCurrent + inputs.followerMotorSupplyCurrent);

    if (DriverStation.isDisabled()) {
      outputs.mode = BedOutputMode.BRAKE;
    }

    if (kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get());
    }

    LoggedTracer.record("Bed");
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Bed/OutputMode", outputs.mode.toString());
    Logger.recordOutput("Bed/GoalRPM", outputs.velocityRPM);
    Logger.recordOutput("Bed/AtSetpoint", isAtSetpoint());
    io.applyOutputs(outputs);
  }

  // --- RPM API ---

  /**
   * Commands the bed rollers to a desired RPM.
   *
   * @param rpm the target roller RPM.
   */
  public void setBedRPM(double rpm) {
    outputs.mode = BedOutputMode.VELOCITY;
    outputs.velocityRPM = rpm;
  }

  /**
   * Returns the current leader roller RPM.
   *
   * @return leader roller RPM.
   */
  public double getBedRPM() {
    return inputs.leaderMotorVelocityRPM;
  }

  /**
   * Returns whether the bed is at the commanded RPM setpoint.
   *
   * @return true if the bed RPM is within tolerance.
   */
  public boolean isAtSetpoint() {
    return Math.abs(inputs.leaderMotorVelocityRPM - outputs.velocityRPM) <= BedConstants.kTolerance;
  }

  // --- Voltage API ---

  /**
   * Sets the bed motor voltage directly (open-loop).
   *
   * @param voltage voltage from -12 to 12.
   */
  public void setVoltage(double voltage) {
    outputs.mode = BedOutputMode.VOLTAGE;
    outputs.volts = voltage;
  }

  /** Stops the bed motors (brake). */
  public void stop() {
    outputs.mode = BedOutputMode.BRAKE;
  }

  /** Set brake mode (true) or coast mode (false). */
  public void setBrakeMode(boolean brake) {
    outputs.brakeMode = brake;
  }
}
