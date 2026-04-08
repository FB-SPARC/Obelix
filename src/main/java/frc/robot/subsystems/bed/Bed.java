package frc.robot.subsystems.bed;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Bed subsystem that controls a bed of rollers for transporting game pieces. */
public class Bed extends SubsystemBase {

  private final BedIO io;
  private final BedIOInputsAutoLogged inputs = new BedIOInputsAutoLogged();

  /** Creates a new Bed. */
  public Bed(BedIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Bed", inputs);
  }

  // --- RPM API ---

  /**
   * Commands the bed rollers to a desired RPM.
   *
   * @param rpm the target roller RPM.
   */
  public void setBedRPM(double rpm) {
    io.setBedRPM(rpm);
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
    return io.isAtSetpoint();
  }

  // --- Voltage API ---

  /**
   * Sets the bed motor voltage directly (open-loop).
   *
   * @param voltage voltage from -12 to 12.
   */
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  /** Stops the bed motors. */
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

  /** Set brake mode (true) or coast mode (false). */
  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }
}
