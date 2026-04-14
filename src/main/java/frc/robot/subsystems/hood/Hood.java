package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.subsystems.hood.HoodIO.HoodIOOutputs;
import frc.robot.subsystems.hood.HoodIO.HoodOutputMode;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

/**
 * Hood subsystem that controls the angle of the shooter hood for shot trajectory adjustment. Uses a
 * CANCoder absolute encoder for position feedback.
 *
 * <p>Follows the FullSubsystem pattern: goals are stored in {@link #outputs} during commands and
 * applied atomically to the IO layer in {@link #periodicAfterScheduler()}.
 */
public class Hood extends FullSubsystem {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  private final Alert motorDisconnectedAlert =
      new Alert("Hood motor disconnected!", AlertType.kError);
  private final Alert encoderDisconnectedAlert =
      new Alert("Hood CANCoder disconnected!", AlertType.kError);

  /** Creates a new Hood. */
  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    motorDisconnectedAlert.set(Robot.showHardwareAlerts() && !inputs.motorConnected);
    encoderDisconnectedAlert.set(Robot.showHardwareAlerts() && !inputs.encoderConnected);

    // Report current draw to battery logger
    Robot.batteryLogger.reportCurrentUsage("Hood", false, inputs.motorCurrent);

    // Brake when disabled
    if (DriverStation.isDisabled()) {
      outputs.mode = HoodOutputMode.BRAKE;
    }

    LoggedTracer.record("Hood");
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Hood/OutputMode", outputs.mode.toString());
    Logger.recordOutput("Hood/GoalDegrees", outputs.positionDegrees);
    Logger.recordOutput("Hood/AtSetpoint", isAtSetpoint());
    io.applyOutputs(outputs);
  }

  // --- Position API (degrees) ---

  /**
   * Commands the hood to a desired angle.
   *
   * @param degrees the target angle in degrees.
   */
  public void setAngle(double degrees) {
    outputs.mode = HoodOutputMode.POSITION;
    outputs.positionDegrees = degrees;
  }

  /**
   * Returns the current hood position in degrees (from FusedCANcoder).
   *
   * @return hood position in degrees.
   */
  public double getAngle() {
    return inputs.mechanismPositionDegrees;
  }

  /**
   * Returns the current hood absolute encoder angle in degrees.
   *
   * @return absolute hood angle in degrees.
   */
  public double getAbsoluteAngle() {
    return inputs.absoluteEncoderPositionDegrees;
  }

  /**
   * Returns the current hood velocity in degrees per second.
   *
   * @return hood velocity in deg/s.
   */
  public double getVelocity() {
    return inputs.motorVelocityDegreesPerSecond;
  }

  /**
   * Returns whether the hood is at the commanded angle setpoint.
   *
   * @return true if the hood is within tolerance.
   */
  public boolean isAtSetpoint() {
    return Math.abs(inputs.mechanismPositionDegrees - outputs.positionDegrees)
        <= HoodConstants.kTolerance;
  }

  // --- Voltage API ---

  /**
   * Sets the hood motor voltage directly (open-loop).
   *
   * @param voltage voltage from -12 to 12.
   */
  public void setVoltage(double voltage) {
    outputs.mode = HoodOutputMode.VOLTAGE;
    outputs.volts = voltage;
  }

  /** Stops the hood motor (brake). */
  public void stop() {
    outputs.mode = HoodOutputMode.BRAKE;
  }

  /** Set brake mode (true) or coast mode (false). */
  public void setBrakeMode(boolean brake) {
    outputs.brakeMode = brake;
  }
}
