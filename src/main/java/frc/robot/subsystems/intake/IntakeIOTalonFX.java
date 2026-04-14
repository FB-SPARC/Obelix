package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/**
 * IntakeIOTalonFX implements the IntakeIO interface using a CTRE TalonFX motor controller. The
 * leader drives open-loop voltage control. No closed-loop RPM control — intake is typically run at
 * a fixed voltage.
 */
public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);

  // Leader status signals
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;

  // Follower status signals
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerCurrent;

  // Connection debouncers
  private final Debouncer leaderConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5);

  public IntakeIOTalonFX() {
    leaderMotor = new TalonFX(IntakeConstants.MOTOR_ID, Constants.canivore);
    followerMotor = new TalonFX(IntakeConstants.FOLLOWER_MOTOR_ID, Constants.canivore);

    // Leader configuration
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.MAX_CURRENT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Apply configuration to leader
    tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> leaderMotor.setPosition(0.0, 0.25));

    // Follower configuration (same limits, no inversion needed — Follower handles opposition)
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.MAX_CURRENT;
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> followerMotor.getConfigurator().apply(followerConfig, 0.25));
    followerMotor.setControl(new Follower(IntakeConstants.MOTOR_ID, MotorAlignmentValue.Opposed));

    // Initialize leader status signals
    leaderVelocity = leaderMotor.getVelocity();
    leaderAppliedVolts = leaderMotor.getMotorVoltage();
    leaderCurrent = leaderMotor.getStatorCurrent();

    // Initialize follower status signals
    followerVelocity = followerMotor.getVelocity();
    followerAppliedVolts = followerMotor.getMotorVoltage();
    followerCurrent = followerMotor.getStatorCurrent();

    // Set update frequencies — on CANivore (CAN FD), use higher rates
    BaseStatusSignal.setUpdateFrequencyForAll(
        200,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        followerVelocity,
        followerAppliedVolts,
        followerCurrent);

    // Optimize CAN bus utilization
    ParentDevice.optimizeBusUtilizationForAll(leaderMotor, followerMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    StatusCode leaderStatus =
        BaseStatusSignal.refreshAll(leaderVelocity, leaderAppliedVolts, leaderCurrent);
    StatusCode followerStatus =
        BaseStatusSignal.refreshAll(followerVelocity, followerAppliedVolts, followerCurrent);

    inputs.leaderMotorConnected = leaderConnectedDebouncer.calculate(leaderStatus.isOK());
    inputs.leaderMotorVelocityRPM = leaderVelocity.getValueAsDouble() * 60.0;
    inputs.leaderMotorVoltage = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderMotorCurrent = leaderCurrent.getValueAsDouble();

    inputs.followerMotorConnected = followerConnectedDebouncer.calculate(followerStatus.isOK());
    inputs.followerMotorVelocityRPM = followerVelocity.getValueAsDouble() * 60.0;
    inputs.followerMotorVoltage = followerAppliedVolts.getValueAsDouble();
    inputs.followerMotorCurrent = followerCurrent.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    leaderMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public double getIntakeRPM() {
    return leaderVelocity.getValueAsDouble() * 60.0;
  }

  @Override
  public double getVelocity() {
    return leaderVelocity.getValueAsDouble() * 360.0;
  }

  @Override
  public double getCurrent() {
    return leaderCurrent.getValueAsDouble();
  }

  @Override
  public double getVoltage() {
    return leaderAppliedVolts.getValueAsDouble();
  }

  @Override
  public boolean isAtSetpoint() {
    return false; // Intake is typically open-loop
  }

  @Override
  public void setIntakeRPM(double rpm) {
    // Open-loop approximation: convert RPM to rough voltage
    // For proper closed-loop, add VelocityVoltage like Bed/Feeder
    double voltage = (rpm / 6380.0) * 12.0; // Falcon 500 free speed ~6380 RPM
    leaderMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void resetEncoder() {
    tryUntilOk(5, () -> leaderMotor.setPosition(0.0, 0.25));
  }

  @Override
  public void setBrakeMode(boolean brake) {
    var mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    leaderMotor.setNeutralMode(mode);
    followerMotor.setNeutralMode(mode);
  }
}
