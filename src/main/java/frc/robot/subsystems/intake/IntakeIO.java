package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean leaderMotorConnected = false;
    public double leaderMotorCurrent = 0.0;
    public double leaderMotorVoltage = 0.0;
    public double leaderMotorVelocityRPM = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  // Sets motor voltage from -12V to 12V
  public default void setVoltage(double voltage) {}

  public default double getIntakeRPM() {
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

  public default void setIntakeRPM(double rpm) {}

  public default void resetEncoder() {}

  /** Set brake mode (true) or coast mode (false). */
  public default void setBrakeMode(boolean brake) {}
}
