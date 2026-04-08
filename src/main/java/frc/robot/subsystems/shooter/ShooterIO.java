package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leaderMotorConnected = false;
    public double leaderMotorCurrent = 0.0;
    public double leaderMotorVoltage = 0.0;
    public double leaderMotorVelocityRPM = 0.0;

    public boolean followerMotor1Connected = false;
    public double followerMotor1Current = 0.0;
    public double followerMotor1Voltage = 0.0;
    public double followerMotor1VelocityRPM = 0.0;

    public boolean followerMotor2Connected = false;
    public double followerMotor2Current = 0.0;
    public double followerMotor2Voltage = 0.0;
    public double followerMotor2VelocityRPM = 0.0;

    public boolean followerMotor3Connected = false;
    public double followerMotor3Current = 0.0;
    public double followerMotor3Voltage = 0.0;
    public double followerMotor3VelocityRPM = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  // Sets motor voltage from -12V to 12V
  public default void setVoltage(double voltage) {}

  public default double getShooterRPM() {
    return 0.0;
  }

  // Returns velocity of the leader motor in RPM
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

  public default void setShooterRPM(double rpm) {}

  public default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public default void resetEncoder() {}

  /** Set brake mode (true) or coast mode (false). */
  public default void setBrakeMode(boolean brake) {}
}
