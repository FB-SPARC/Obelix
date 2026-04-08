package frc.robot.subsystems.rack;

public final class RackConstants {
  public static final int MOTOR_ID = 15; // TODO: Set CAN ID

  public static final double kGearRatio = 169.0 / 48.0;
  public static final double kPinionRadiusMeters = 0.02032;

  public static final double MAX_CURRENT = 100.0; // Amps

  public static final double MAX_POSITION_METERS = 0.28;
  public static final double MIN_POSITION_METERS = 0.0;

  public static final double kTolerance = 0.05; // Meters

  // PID + feedforward gains
  public static final double kP = 7.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  // Motion Magic profile
  public static final double kCruiseVelocity = 5.0; // rot/s
  public static final double kAcceleration = 15.0; // rot/s^2
  public static final double kJerk = 800.0; // rot/s^3
}
