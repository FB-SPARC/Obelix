// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import frc.robot.subsystems.intake.IntakeIO.IntakeOutputMode;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTracer;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem that controls roller motors for game piece acquisition.
 *
 * <p>Follows the FullSubsystem pattern: goals are stored in {@link #outputs} during commands and
 * applied atomically to the IO layer in {@link #periodicAfterScheduler()}.
 */
public class Intake extends FullSubsystem {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOOutputs outputs = new IntakeIOOutputs();

  // ── Tunable PID/FF gains (for future closed-loop intake control) ──────────
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Intake/kP", IntakeConstants.kP);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Intake/kI", IntakeConstants.kI);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Intake/kD", IntakeConstants.kD);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Intake/kS", IntakeConstants.kS);
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Intake/kV", IntakeConstants.kV);
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Intake/kA", IntakeConstants.kA);

  private final Alert leaderDisconnectedAlert =
      new Alert("Intake leader motor disconnected!", AlertType.kError);
  private final Alert followerDisconnectedAlert =
      new Alert("Intake follower motor disconnected!", AlertType.kError);

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    leaderDisconnectedAlert.set(Robot.showHardwareAlerts() && !inputs.leaderMotorConnected);
    followerDisconnectedAlert.set(Robot.showHardwareAlerts() && !inputs.followerMotorConnected);

    Robot.batteryLogger.reportCurrentUsage(
        "Intake", false, inputs.leaderMotorSupplyCurrent + inputs.followerMotorSupplyCurrent);

    if (DriverStation.isDisabled()) {
      outputs.mode = IntakeOutputMode.BRAKE;
    }

    if (kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get());
    }

    LoggedTracer.record("Intake");
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Intake/OutputMode", outputs.mode.toString());
    Logger.recordOutput("Intake/VelocitySetpointRPM", outputs.velocityRPM);
    io.applyOutputs(outputs);
  }

  // --- Voltage API ---

  /**
   * Sets the intake motor voltage directly (open-loop).
   *
   * @param voltage voltage from -12 to 12.
   */
  public void setVoltage(double voltage) {
    outputs.mode = IntakeOutputMode.VOLTAGE;
    outputs.volts = voltage;
  }

  // --- Velocity (RPM) API ---

  /**
   * Sets the intake motor to a closed-loop velocity setpoint.
   *
   * @param rpm target RPM (positive = intake, negative = eject).
   */
  public void setRPM(double rpm) {
    outputs.mode = IntakeOutputMode.VELOCITY;
    outputs.velocityRPM = rpm;
  }

  /**
   * Returns the current leader roller RPM.
   *
   * @return leader roller RPM.
   */
  public double getIntakeRPM() {
    return inputs.leaderMotorVelocityRPM;
  }

  /** Stops the intake motors (brake). */
  public void stop() {
    outputs.mode = IntakeOutputMode.BRAKE;
  }

  /** Set brake mode (true) or coast mode (false). */
  public void setBrakeMode(boolean brake) {
    outputs.brakeMode = brake;
  }
}
