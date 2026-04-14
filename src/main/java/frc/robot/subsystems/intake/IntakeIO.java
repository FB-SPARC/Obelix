package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean leaderMotorConnected = false;
    public double leaderMotorCurrent = 0.0;
    public double leaderMotorVoltage = 0.0;
    public double leaderMotorVelocityRPM = 0.0;

    public boolean followerMotorConnected = false;
    public double followerMotorCurrent = 0.0;
    public double followerMotorVoltage = 0.0;
    public double followerMotorVelocityRPM = 0.0;
  }

  public static enum IntakeOutputMode {
    VOLTAGE,
    BRAKE
  }

  public static class IntakeIOOutputs {
    public IntakeOutputMode mode = IntakeOutputMode.BRAKE;
    public double volts = 0.0;
    public boolean brakeMode = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void applyOutputs(IntakeIOOutputs outputs) {}
}
