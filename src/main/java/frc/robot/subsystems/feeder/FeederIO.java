package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public boolean leaderMotorConnected = false;
    public double leaderMotorCurrent = 0.0;
    public double leaderMotorVoltage = 0.0;
    public double leaderMotorVelocityRPM = 0.0;

    public boolean followerMotorConnected = false;
    public double followerMotorCurrent = 0.0;
    public double followerMotorVoltage = 0.0;
    public double followerMotorVelocityRPM = 0.0;

    public double feederSetpointRPM = 0.0;
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  // Sets motor voltage from -12V to 12V
  public default void setVoltage(double voltage) {}

  public default double getFeederRPM() {
    return 0.0;
  }

  // Returns velocity of the leader motor in degrees per second
  public default double getVelocity() {
    return 0.0;
  }

  // Returns current of the leader motor in amps
  public default double getCurrent() {
    return 0.0;
  }

  // Returns voltage of the leader motor in volts
  public default double getVoltage() {
    return 0.0;
  }

  public default boolean isAtSetpoint() {
    return false;
  }

  public default void setFeederRPM(double rpm) {}

  public default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public default void resetEncoder() {}

  /** Set brake mode (true) or coast mode (false). */
  public default void setBrakeMode(boolean brake) {}
}
