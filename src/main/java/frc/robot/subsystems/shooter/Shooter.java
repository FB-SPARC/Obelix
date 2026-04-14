package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOOutputs;
import frc.robot.subsystems.shooter.ShooterIO.ShooterOutputMode;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem that controls a drum shooter with 4 motors (1 leader + 3 followers). Provides
 * surface velocity control in meters per second, converting to drum RPM using the drum radius
 * defined in {@link ShooterConstants}.
 *
 * <p>Follows the FullSubsystem pattern: goals are stored in {@link #outputs} during commands and
 * applied atomically to the IO layer in {@link #periodicAfterScheduler()}.
 */
public class Shooter extends FullSubsystem {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIOOutputs outputs = new ShooterIOOutputs();

  /** Drum circumference in meters, used for surface velocity conversions. */
  private static final double kDrumCircumferenceMeters =
      2.0 * Math.PI * ShooterConstants.kDrumRadius.in(Units.Meters);

  private final Alert leaderDisconnectedAlert =
      new Alert("Shooter leader motor disconnected!", AlertType.kError);
  private final Alert follower1DisconnectedAlert =
      new Alert("Shooter follower 1 disconnected!", AlertType.kError);
  private final Alert follower2DisconnectedAlert =
      new Alert("Shooter follower 2 disconnected!", AlertType.kError);
  private final Alert follower3DisconnectedAlert =
      new Alert("Shooter follower 3 disconnected!", AlertType.kError);

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    leaderDisconnectedAlert.set(Robot.showHardwareAlerts() && !inputs.leaderMotorConnected);
    follower1DisconnectedAlert.set(Robot.showHardwareAlerts() && !inputs.followerMotor1Connected);
    follower2DisconnectedAlert.set(Robot.showHardwareAlerts() && !inputs.followerMotor2Connected);
    follower3DisconnectedAlert.set(Robot.showHardwareAlerts() && !inputs.followerMotor3Connected);

    Robot.batteryLogger.reportCurrentUsage(
        "Shooter",
        false,
        inputs.leaderMotorCurrent
            + inputs.followerMotor1Current
            + inputs.followerMotor2Current
            + inputs.followerMotor3Current);

    if (DriverStation.isDisabled()) {
      outputs.mode = ShooterOutputMode.BRAKE;
    }

    LoggedTracer.record("Shooter");
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Shooter/OutputMode", outputs.mode.toString());
    Logger.recordOutput("Shooter/GoalRPM", outputs.velocityRPM);
    Logger.recordOutput("Shooter/GoalSurfaceVelocity", rpmToSurfaceVelocity(outputs.velocityRPM));
    Logger.recordOutput("Shooter/AtSetpoint", isAtSetpoint());
    io.applyOutputs(outputs);
  }

  // --- Surface velocity API (meters per second) ---

  /**
   * Commands the shooter to a desired surface velocity.
   *
   * @param metersPerSecond the target exit speed in m/s.
   */
  public void setSurfaceVelocity(double metersPerSecond) {
    outputs.mode = ShooterOutputMode.VELOCITY;
    outputs.velocityRPM = surfaceVelocityToRPM(metersPerSecond);
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
   * Returns whether the shooter is at the commanded RPM/surface-velocity setpoint.
   *
   * @return true if the shooter RPM is within tolerance.
   */
  public boolean isAtSetpoint() {
    return Math.abs(inputs.leaderMotorVelocityRPM - outputs.velocityRPM)
        <= ShooterConstants.kTolerance;
  }

  // --- RPM API ---

  /**
   * Commands the shooter to a desired RPM directly.
   *
   * @param rpm the target drum RPM.
   */
  public void setShooterRPM(double rpm) {
    outputs.mode = ShooterOutputMode.VELOCITY;
    outputs.velocityRPM = rpm;
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
    outputs.mode = ShooterOutputMode.VOLTAGE;
    outputs.volts = voltage;
  }

  /** Stops the shooter motors (brake). */
  public void stop() {
    outputs.mode = ShooterOutputMode.BRAKE;
  }

  /** Set brake mode (true) or coast mode (false). */
  public void setBrakeMode(boolean brake) {
    outputs.brakeMode = brake;
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
}
