package frc.robot.subsystems.hood;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/**
 * HoodIOTalonFX implements the HoodIO interface using a CTRE TalonFX motor controller and a
 * CANCoder absolute encoder. Uses a single PID slot with Motion Magic for position control.
 */
public class HoodIOTalonFX implements HoodIO {

  private final TalonFX motor;
  private final CANcoder encoder;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withEnableFOC(true);

  // Motor status signals
  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;

  // Encoder status signals
  private final StatusSignal<Angle> encoderAbsolutePosition;

  // Connection debouncers
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer encoderConnectedDebouncer = new Debouncer(0.5);

  // Setpoint tracking
  private double setpointDegrees = 0.0;

  public HoodIOTalonFX() {
    motor = new TalonFX(HoodConstants.MOTOR_ID, Constants.canivore);
    encoder = new CANcoder(HoodConstants.CANCODER_ID, Constants.canivore);

    // --- CANCoder configuration ---
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset =
        HoodConstants.kEncoderOffset / 360.0; // degrees to rotations
    tryUntilOk(5, () -> encoder.getConfigurator().apply(encoderConfig, 0.25));

    // --- TalonFX configuration ---
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = HoodConstants.MAX_CURRENT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Soft limits (in mechanism rotations — fused sensor reports mechanism units)
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(HoodConstants.MAX_ANGLE_DEGREES);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(HoodConstants.MIN_ANGLE_DEGREES);

    // Fused CANCoder feedback — PID runs on the CANCoder, not the motor encoder
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.FeedbackRemoteSensorID = HoodConstants.CANCODER_ID;
    config.Feedback.RotorToSensorRatio = HoodConstants.kGearRatio; // 4:1 motor-to-CANCoder
    config.Feedback.SensorToMechanismRatio = HoodConstants.kSensorToMechanismRatio;

    // PID + feedforward for position control
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = HoodConstants.kP;
    slot0.kI = HoodConstants.kI;
    slot0.kD = HoodConstants.kD;
    slot0.kS = HoodConstants.kS;
    slot0.kV = HoodConstants.kV;
    slot0.kA = HoodConstants.kA;

    // Motion Magic configuration
    MotionMagicConfigs motionMagic = config.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = HoodConstants.kCruiseVelocity;
    motionMagic.MotionMagicAcceleration = HoodConstants.kAcceleration;
    motionMagic.MotionMagicJerk = HoodConstants.kJerk;

    // Apply configuration (do NOT zero encoder — FusedCANcoder derives position from the CANcoder)
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    // Initialize motor status signals
    motorPosition = motor.getPosition();
    motorVelocity = motor.getVelocity();
    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();

    // Initialize encoder status signals
    encoderAbsolutePosition = encoder.getAbsolutePosition();

    // Set update frequencies — on CANivore (CAN FD), use higher rates
    BaseStatusSignal.setUpdateFrequencyForAll(
        200,
        motorPosition,
        motorVelocity,
        motorAppliedVolts,
        motorCurrent,
        encoderAbsolutePosition);

    // Optimize CAN bus utilization
    ParentDevice.optimizeBusUtilizationForAll(motor, encoder);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    StatusCode motorStatus =
        BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);
    StatusCode encoderStatus = BaseStatusSignal.refreshAll(encoderAbsolutePosition);

    inputs.motorConnected = motorConnectedDebouncer.calculate(motorStatus.isOK());
    inputs.motorPositionDegrees = getMotorPositionDegrees();
    inputs.motorVelocityDegreesPerSecond =
        Units.rotationsToDegrees(motorVelocity.getValueAsDouble());
    inputs.motorVoltage = motorAppliedVolts.getValueAsDouble();
    inputs.motorCurrent = motorCurrent.getValueAsDouble();
    inputs.mechanismPositionDegrees = getHoodPositionDegrees();

    inputs.encoderConnected = encoderConnectedDebouncer.calculate(encoderStatus.isOK());
    inputs.absoluteEncoderPositionDegrees =
        Units.rotationsToDegrees(encoderAbsolutePosition.getValueAsDouble());
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public double getMotorPositionDegrees() {
    return Units.rotationsToDegrees(motorPosition.getValueAsDouble());
  }

  @Override
  public double getHoodPositionDegrees() {
    // With FusedCANcoder, motor.getPosition() reports mechanism rotations directly
    return Units.rotationsToDegrees(motorPosition.getValueAsDouble());
  }

  @Override
  public double getVelocity() {
    // With FusedCANcoder, motor.getVelocity() reports mechanism rot/s directly
    return Units.rotationsToDegrees(motorVelocity.getValueAsDouble());
  }

  @Override
  public double getCurrent() {
    return motorCurrent.getValueAsDouble();
  }

  @Override
  public double getVoltage() {
    return motorAppliedVolts.getValueAsDouble();
  }

  @Override
  public void setHoodPositionDegrees(double degrees) {
    this.setpointDegrees = degrees;

    // With FusedCANcoder, position target is in mechanism rotations
    double mechanismRotations = Units.degreesToRotations(degrees);

    motor.setControl(motionMagicRequest.withPosition(mechanismRotations));
  }

  @Override
  public boolean isAtSetpoint() {
    return Math.abs(setpointDegrees - getHoodPositionDegrees()) <= HoodConstants.kTolerance;
  }

  @Override
  public void resetEncoder() {
    // No-op: FusedCANcoder derives position from the absolute CANcoder.
    // Zeroing the motor position would be immediately overridden.
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
  public void setBrakeMode(boolean brake) {
    motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
