package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import frc.robot.subsystems.intake.IntakeIO.IntakeOutputMode;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTracer;
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
        "Intake", false, inputs.leaderMotorCurrent + inputs.followerMotorCurrent);

    if (DriverStation.isDisabled()) {
      outputs.mode = IntakeOutputMode.BRAKE;
    }

    LoggedTracer.record("Intake");
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Intake/OutputMode", outputs.mode.toString());
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
