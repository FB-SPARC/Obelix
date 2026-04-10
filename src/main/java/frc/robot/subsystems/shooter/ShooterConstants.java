package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public final class ShooterConstants {
  public static final int LEADER_MOTOR_ID = 50; // Left up
  public static final int FOLLOWER_MOTOR_1_ID = 51; // Left down
  public static final int FOLLOWER_MOTOR_2_ID = 52; // Right up
  public static final int FOLLOWER_MOTOR_3_ID = 53; // Right down

  public static final Distance kDrumRadius = Meters.of(0.0508);

  public static final double MAX_CURRENT = 100.0; // Amps

  public static final double kTolerance = 25; // RPM

  public static final double kP = 0.4;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.125;
  public static final double kA = 0.1;
}
