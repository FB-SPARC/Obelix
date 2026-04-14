package frc.robot.subsystems.rack;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.subsystems.rack.RackIO.RackIOOutputs;
import frc.robot.subsystems.rack.RackIO.RackOutputMode;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

/**
 * Rack subsystem that controls a rack and pinion mechanism for the intake deployment. Provides
 * position control in meters.
 *
 * <p>Follows the FullSubsystem pattern: goals are stored in {@link #outputs} during commands and
 * applied atomically to the IO layer in {@link #periodicAfterScheduler()}.
 */
public class Rack extends FullSubsystem {

  private final RackIO io;
  private final RackIOInputsAutoLogged inputs = new RackIOInputsAutoLogged();
  private final RackIOOutputs outputs = new RackIOOutputs();

  private final Alert motorDisconnectedAlert =
      new Alert("Rack motor disconnected!", AlertType.kError);

  /** Creates a new Rack. */
  public Rack(RackIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rack", inputs);

    motorDisconnectedAlert.set(Robot.showHardwareAlerts() && !inputs.motorConnected);

    Robot.batteryLogger.reportCurrentUsage("Rack", false, inputs.motorCurrent);

    if (DriverStation.isDisabled()) {
      outputs.mode = RackOutputMode.BRAKE;
    }

    LoggedTracer.record("Rack");
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Rack/OutputMode", outputs.mode.toString());
    Logger.recordOutput("Rack/GoalPositionMeters", outputs.positionMeters);
    Logger.recordOutput("Rack/AtSetpoint", isAtSetpoint());
    io.applyOutputs(outputs);
  }

  // --- Position API (meters) ---

  /**
   * Commands the rack to a desired linear position with explicit motion-magic gains.
   *
   * @param meters the target position in meters.
   * @param kv velocity feedforward gain (V·s/m).
   * @param ka acceleration feedforward gain (V·s²/m).
   * @param kj jerk feedforward gain (V·s³/m).
   */
  public void setPosition(double meters, double kv, double ka, double kj) {
    outputs.mode = RackOutputMode.POSITION;
    outputs.positionMeters = meters;
    outputs.kv = kv;
    outputs.ka = ka;
    outputs.kj = kj;
  }

  /**
   * Returns the current rack linear position in meters.
   *
   * @return rack position in meters.
   */
  public double getPosition() {
    return inputs.rackPositionMeters;
  }

  /**
   * Returns the current motor position in degrees.
   *
   * @return motor position in degrees.
   */
  public double getMotorPositionDegrees() {
    return inputs.motorPositionDegrees;
  }

  /**
   * Returns the current motor velocity in degrees per second.
   *
   * @return motor velocity in deg/s.
   */
  public double getVelocity() {
    return inputs.motorVelocityDegreesPerSecond;
  }

  /**
   * Returns whether the rack is at the commanded position setpoint.
   *
   * @return true if the rack is within tolerance.
   */
  public boolean isAtSetpoint() {
    return Math.abs(inputs.rackPositionMeters - outputs.positionMeters) <= RackConstants.kTolerance;
  }

  // --- Voltage API ---

  /**
   * Sets the rack motor voltage directly (open-loop).
   *
   * @param voltage voltage from -12 to 12.
   */
  public void setVoltage(double voltage) {
    outputs.mode = RackOutputMode.VOLTAGE;
    outputs.volts = voltage;
  }

  /** Stops the rack motor (brake). */
  public void stop() {
    outputs.mode = RackOutputMode.BRAKE;
  }

  /** Set brake mode (true) or coast mode (false). */
  public void setBrakeMode(boolean brake) {
    outputs.brakeMode = brake;
  }
}
