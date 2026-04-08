package frc.robot.subsystems.rack;

import org.littletonrobotics.junction.AutoLog;

public interface RackIO {
  @AutoLog
  public static class RackIOInputs {
    public boolean motorConnected = false;
    public double motorCurrent = 0.0;
    public double motorVoltage = 0.0;
    public double motorPositionDegrees = 0.0;
    public double motorVelocityDegreesPerSecond = 0.0;
    public double rackPositionMeters = 0.0;
    public double mechanismPositionMeters = 0.0;
  }

  public default void updateInputs(RackIOInputs inputs) {}

  // Sets motor voltage from -12V to 12V
  public default void setVoltage(double voltage) {}

  public default double getMotorPositionDegrees() {
    return 0.0;
  }

  // Returns the linear position of the rack in meters
  public default double getRackPositionMeters() {
    return 0.0;
  }

  // Returns velocity of the motor in degrees per second
  public default double getVelocity() {
    return 0.0;
  }

  // Returns current of the motor in amps
  public default double getCurrent() {
    return 0.0;
  }

  // Returns voltage of the motor in volts
  public default double getVoltage() {
    return 0.0;
  }

  public default boolean isAtSetpoint() {
    return true;
  }

  // Sets the rack to a target linear position in meters
  public default void setRackPositionMeters(double meters, double kv, double ka, double kj) {}

  public default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public default void resetEncoder() {}

  /** Set brake mode (true) or coast mode (false). */
  public default void setBrakeMode(boolean brake) {}
}
