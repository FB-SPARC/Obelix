package frc.robot.subsystems.rack;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Rack subsystem that controls a rack and pinion mechanism for the intake deployment. Provides
 * position control in meters.
 */
public class Rack extends SubsystemBase {

  private final RackIO io;
  private final RackIOInputsAutoLogged inputs = new RackIOInputsAutoLogged();

  /** Creates a new Rack. */
  public Rack(RackIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rack", inputs);
    Logger.recordOutput("Rack/isAtSetpoint", isAtSetpoint());
  }

  // --- Position API (meters) ---

  /**
   * Commands the rack to a desired linear position.
   *
   * @param meters the target position in meters.
   */
  public void setPosition(double meters, double kv, double ka, double kj) {
    io.setRackPositionMeters(meters, kv, ka, kj);
  }

  /**
   * Returns the current rack linear position in meters.
   *
   * @return rack position in meters.
   */
  public double getPosition() {
    return inputs.rackPositionMeters;
  }

  /**
   * Returns the current motor position in degrees.
   *
   * @return motor position in degrees.
   */
  public double getMotorPositionDegrees() {
    return inputs.motorPositionDegrees;
  }

  /**
   * Returns the current motor velocity in degrees per second.
   *
   * @return motor velocity in deg/s.
   */
  public double getVelocity() {
    return inputs.motorVelocityDegreesPerSecond;
  }

  /**
   * Returns whether the rack is at the commanded position setpoint.
   *
   * @return true if the rack is within tolerance.
   */
  public boolean isAtSetpoint() {
    return io.isAtSetpoint();
  }

  // --- Voltage API ---

  /**
   * Sets the rack motor voltage directly (open-loop).
   *
   * @param voltage voltage from -12 to 12.
   */
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  /** Stops the rack motor. */
  public void stop() {
    io.setVoltage(0.0);
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

  /** Resets the rack encoder to zero. */
  public void resetEncoder() {
    io.resetEncoder();
  }

  /** Set brake mode (true) or coast mode (false). */
  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }
}
