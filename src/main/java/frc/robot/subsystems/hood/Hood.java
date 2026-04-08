package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Hood subsystem that controls the angle of the shooter hood for shot trajectory adjustment. Uses a
 * CANCoder absolute encoder for position feedback.
 */
public class Hood extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  /** Creates a new Hood. */
  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  // --- Position API (degrees) ---

  /**
   * Commands the hood to a desired angle.
   *
   * @param degrees the target angle in degrees.
   */
  public void setAngle(double degrees) {
    io.setHoodPositionDegrees(degrees);
  }

  /**
   * Returns the current hood angle in degrees from the motor encoder (after gear ratio).
   *
   * @return hood angle in degrees.
   */
  public double getAngle() {
    return inputs.motorPositionDegrees;
  }

  /**
   * Returns the current hood mechanism position in degrees (FusedCANcoder output).
   *
   * @return mechanism position in degrees.
   */
  public double getMechanismPosition() {
    return inputs.mechanismPositionDegrees;
  }

  /**
   * Returns the current hood angle in degrees from the CANCoder absolute encoder.
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
    return io.isAtSetpoint();
  }

  // --- Voltage API ---

  /**
   * Sets the hood motor voltage directly (open-loop).
   *
   * @param voltage voltage from -12 to 12.
   */
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  /** Stops the hood motor. */
  public void stop() {
    io.setVoltage(0.0);
  }

  /** Resets the hood encoder to zero. */
  public void resetEncoder() {
    io.resetEncoder();
  }

  /**
   * Updates the PID and feedforward gains on the motor controller at runtime.
   *
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param kS static feedforward
   * @param kV velocity feedforward
   * @param kA acceleration feedforward
   */
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {
    io.setPID(kP, kI, kD, kS, kV, kA);
  }

  /** Set brake mode (true) or coast mode (false). */
  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }
}
