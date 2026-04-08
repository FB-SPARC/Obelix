package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Feeder subsystem that controls roller motors for transporting game pieces to the shooter. */
public class Feeder extends SubsystemBase {

  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  /** Creates a new Feeder. */
  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  // --- RPM API ---

  /**
   * Commands the feeder to a desired RPM.
   *
   * @param rpm the target roller RPM.
   */
  public void setFeederRPM(double rpm) {
    io.setFeederRPM(rpm);
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
    return io.isAtSetpoint();
  }

  // --- Voltage API ---

  /**
   * Sets the feeder motor voltage directly (open-loop).
   *
   * @param voltage voltage from -12 to 12.
   */
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  /** Stops the feeder motors. */
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
