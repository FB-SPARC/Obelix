package frc.robot.subsystems.rack;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/**
 * RackIOTalonFX implements the RackIO interface using a CTRE TalonFX motor controller. It manages a
 * rack and pinion mechanism with Motion Magic position control. Motor rotations are converted to
 * linear meters via the pinion radius and gear ratio.
 */
public class RackIOTalonFX implements RackIO {

  private final TalonFX motor;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final DynamicMotionMagicVoltage motionMagicRequest =
      new DynamicMotionMagicVoltage(0, RackConstants.kCruiseVelocity, RackConstants.kAcceleration)
          .withEnableFOC(true);

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  // Connection debouncer
  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  // Setpoint tracking
  private double setpointMeters = 0.0;

  /** Pinion circumference in meters — one motor rotation moves the rack this distance. */
  private static final double kPinionCircumferenceMeters =
      2.0 * Math.PI * RackConstants.kPinionRadiusMeters;

  public RackIOTalonFX() {
    motor = new TalonFX(RackConstants.MOTOR_ID, Constants.canivore);

    // Build configuration
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = RackConstants.MAX_CURRENT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Soft limits (convert meters → motor rotations)
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        metersToMotorRotations(RackConstants.MAX_POSITION_METERS);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        metersToMotorRotations(RackConstants.MIN_POSITION_METERS);

    // PID + feedforward for position control
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = RackConstants.kP;
    slot0.kI = RackConstants.kI;
    slot0.kD = RackConstants.kD;
    slot0.kS = RackConstants.kS;
    slot0.kV = RackConstants.kV;
    slot0.kA = RackConstants.kA;

    // Motion Magic configuration
    MotionMagicConfigs motionMagic = config.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = RackConstants.kCruiseVelocity;
    motionMagic.MotionMagicAcceleration = RackConstants.kAcceleration;
    motionMagic.MotionMagicJerk = RackConstants.kJerk;

    // Apply configuration and zero encoder
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> motor.setPosition(0.0, 0.25));

    // Initialize status signals
    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getStatorCurrent();

    // Set update frequencies — on CANivore (CAN FD), use higher rates; rack moves fast
    BaseStatusSignal.setUpdateFrequencyForAll(200, position, velocity, appliedVolts, current);

    // Optimize CAN bus utilization
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(RackIOInputs inputs) {
    StatusCode status = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

    inputs.motorConnected = connectedDebouncer.calculate(status.isOK());
    inputs.motorPositionDegrees =
        Units.rotationsToDegrees(position.getValueAsDouble()) / RackConstants.kGearRatio;
    inputs.motorVelocityDegreesPerSecond =
        Units.rotationsToDegrees(velocity.getValueAsDouble()) / RackConstants.kGearRatio;
    inputs.rackPositionMeters = motorRotationsToMeters(position.getValueAsDouble());
    inputs.mechanismPositionMeters = motorRotationsToMeters(position.getValueAsDouble());
    inputs.motorVoltage = appliedVolts.getValueAsDouble();
    inputs.motorCurrent = current.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public double getMotorPositionDegrees() {
    return Units.rotationsToDegrees(position.getValueAsDouble());
  }

  @Override
  public double getRackPositionMeters() {
    return motorRotationsToMeters(position.getValueAsDouble());
  }

  @Override
  public double getVelocity() {
    return Units.rotationsToDegrees(velocity.getValueAsDouble()) / RackConstants.kGearRatio;
  }

  @Override
  public double getCurrent() {
    return current.getValueAsDouble();
  }

  @Override
  public double getVoltage() {
    return appliedVolts.getValueAsDouble();
  }

  @Override
  public void setRackPositionMeters(double meters, double kv, double ka, double kj) {
    this.setpointMeters = meters;

    double motorRotations = metersToMotorRotations(meters);

    motor.setControl(
        motionMagicRequest
            .withPosition(motorRotations)
            .withVelocity(kv)
            .withAcceleration(ka)
            .withJerk(kj));
  }

  @Override
  public boolean isAtSetpoint() {
    return Math.abs(setpointMeters - getRackPositionMeters()) <= RackConstants.kTolerance;
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    motor.getConfigurator().apply(slot0);
  }

  @Override
  public void resetEncoder() {
    tryUntilOk(5, () -> motor.setPosition(0.0, 0.25));
  }

  // --- Conversion helpers ---

  /** Converts linear meters to motor rotations (accounting for gear ratio and pinion). */
  private double metersToMotorRotations(double meters) {
    // meters → mechanism rotations → motor rotations
    double mechanismRotations = meters / kPinionCircumferenceMeters;
    return mechanismRotations * RackConstants.kGearRatio;
  }

  /** Converts motor rotations to linear meters (accounting for gear ratio and pinion). */
  private double motorRotationsToMeters(double motorRotations) {
    double mechanismRotations = motorRotations / RackConstants.kGearRatio;
    return mechanismRotations * kPinionCircumferenceMeters;
  }

  @Override
  public void setBrakeMode(boolean brake) {
    motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
