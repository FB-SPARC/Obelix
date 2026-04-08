package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem that controls roller motors for game piece acquisition. */
public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  // --- RPM API ---

  /**
   * Commands the intake to a desired RPM.
   *
   * @param rpm the target roller RPM.
   */
  public void setIntakeRPM(double rpm) {
    io.setIntakeRPM(rpm);
  }

  /**
   * Returns the current leader roller RPM.
   *
   * @return leader roller RPM.
   */
  public double getIntakeRPM() {
    return inputs.leaderMotorVelocityRPM;
  }

  /**
   * Returns whether the intake is at the commanded RPM setpoint.
   *
   * @return true if the intake RPM is within tolerance.
   */
  public boolean isAtSetpoint() {
    return io.isAtSetpoint();
  }

  // --- Voltage API ---

  /**
   * Sets the intake motor voltage directly (open-loop).
   *
   * @param voltage voltage from -12 to 12.
   */
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  /** Stops the intake motors. */
  public void stop() {
    io.setVoltage(0.0);
  }

  /** Set brake mode (true) or coast mode (false). */
  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }
}
