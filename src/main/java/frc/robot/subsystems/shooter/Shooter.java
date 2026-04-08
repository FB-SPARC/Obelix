package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem that controls a drum shooter with 4 motors (1 leader + 3 followers). Provides
 * surface velocity control in meters per second, converting to drum RPM using the drum radius
 * defined in {@link ShooterConstants}.
 */
public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  /** Drum circumference in meters, used for surface velocity conversions. */
  private static final double kDrumCircumferenceMeters =
      2.0 * Math.PI * ShooterConstants.kDrumRadius.in(Units.Meters);

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  // --- Surface velocity API (meters per second) ---

  /**
   * Commands the shooter to a desired surface velocity.
   *
   * @param metersPerSecond the target exit speed in m/s.
   */
  public void setSurfaceVelocity(double metersPerSecond) {
    double rpm = surfaceVelocityToRPM(metersPerSecond);
    io.setShooterRPM(rpm);
  }

  /**
   * Returns the current surface velocity of the shooter drum in m/s.
   *
   * @return surface velocity in m/s.
   */
  public double getSurfaceVelocity() {
    return rpmToSurfaceVelocity(inputs.leaderMotorVelocityRPM);
  }

  /**
   * Returns whether the shooter is at the commanded surface velocity setpoint.
   *
   * @return true if the shooter RPM is within tolerance.
   */
  public boolean isAtSetpoint() {
    return io.isAtSetpoint();
  }

  // --- RPM API (pass-through for direct control) ---

  /**
   * Commands the shooter to a desired RPM directly.
   *
   * @param rpm the target drum RPM.
   */
  public void setShooterRPM(double rpm) {
    io.setShooterRPM(rpm);
  }

  /**
   * Returns the current drum RPM.
   *
   * @return drum RPM.
   */
  public double getShooterRPM() {
    return inputs.leaderMotorVelocityRPM;
  }

  // --- Voltage API ---

  /**
   * Sets the shooter motor voltage directly (open-loop).
   *
   * @param voltage voltage from -12 to 12.
   */
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  /** Stops the shooter motors. */
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

  // --- Conversion helpers ---

  /**
   * Converts surface velocity (m/s) to drum RPM.
   *
   * @param metersPerSecond surface velocity in m/s.
   * @return drum RPM.
   */
  public static double surfaceVelocityToRPM(double metersPerSecond) {
    return (metersPerSecond * 60.0) / kDrumCircumferenceMeters;
  }

  /**
   * Converts drum RPM to surface velocity (m/s).
   *
   * @param rpm drum RPM.
   * @return surface velocity in m/s.
   */
  public static double rpmToSurfaceVelocity(double rpm) {
    return (rpm / 60.0) * kDrumCircumferenceMeters;
  }

  /** Set brake mode (true) or coast mode (false). */
  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }
}
