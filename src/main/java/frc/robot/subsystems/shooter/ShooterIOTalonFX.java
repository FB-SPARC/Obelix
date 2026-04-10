package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
 * ShooterIOTalonFX implements the ShooterIO interface using four CTRE TalonFX motor controllers in
 * a 1-leader + 3-follower arrangement for a drum shooter.
 */
public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX leaderMotor;
  private final TalonFX followerMotor1;
  private final TalonFX followerMotor2;
  private final TalonFX followerMotor3;

  // Control requests (leader only)
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true);

  // Leader status signals
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;

  // Follower 1 status signals
  private final StatusSignal<AngularVelocity> follower1Velocity;
  private final StatusSignal<Voltage> follower1AppliedVolts;
  private final StatusSignal<Current> follower1Current;

  // Follower 2 status signals
  private final StatusSignal<AngularVelocity> follower2Velocity;
  private final StatusSignal<Voltage> follower2AppliedVolts;
  private final StatusSignal<Current> follower2Current;

  // Follower 3 status signals
  private final StatusSignal<AngularVelocity> follower3Velocity;
  private final StatusSignal<Voltage> follower3AppliedVolts;
  private final StatusSignal<Current> follower3Current;

  // Connection debouncers
  private final Debouncer leaderDebouncer = new Debouncer(0.5);
  private final Debouncer follower1Debouncer = new Debouncer(0.5);
  private final Debouncer follower2Debouncer = new Debouncer(0.5);
  private final Debouncer follower3Debouncer = new Debouncer(0.5);

  // Setpoint tracking
  private double setpointRPM = 0.0;

  public ShooterIOTalonFX() {
    leaderMotor = new TalonFX(ShooterConstants.LEADER_MOTOR_ID, Constants.canivore);
    followerMotor1 = new TalonFX(ShooterConstants.FOLLOWER_MOTOR_1_ID, Constants.canivore);
    followerMotor2 = new TalonFX(ShooterConstants.FOLLOWER_MOTOR_2_ID, Constants.canivore);
    followerMotor3 = new TalonFX(ShooterConstants.FOLLOWER_MOTOR_3_ID, Constants.canivore);

    // --- Leader configuration ---
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leaderConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.MAX_CURRENT;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    Slot0Configs slot0 = leaderConfig.Slot0;
    slot0.kP = ShooterConstants.kP;
    slot0.kI = ShooterConstants.kI;
    slot0.kD = ShooterConstants.kD;
    slot0.kS = ShooterConstants.kS;
    slot0.kV = ShooterConstants.kV;
    slot0.kA = ShooterConstants.kA;

    tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(leaderConfig, 0.25));

    // --- Follower configuration ---
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.MAX_CURRENT;
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> followerMotor1.getConfigurator().apply(followerConfig, 0.25));
    tryUntilOk(5, () -> followerMotor2.getConfigurator().apply(followerConfig, 0.25));
    tryUntilOk(5, () -> followerMotor3.getConfigurator().apply(followerConfig, 0.25));

    // Set all followers to follow the leader (not opposing)
    followerMotor1.setControl(
        new Follower(ShooterConstants.LEADER_MOTOR_ID, MotorAlignmentValue.Aligned));
    followerMotor2.setControl(
        new Follower(ShooterConstants.LEADER_MOTOR_ID, MotorAlignmentValue.Opposed));
    followerMotor3.setControl(
        new Follower(ShooterConstants.LEADER_MOTOR_ID, MotorAlignmentValue.Opposed));

    // --- Initialize status signals ---
    leaderVelocity = leaderMotor.getVelocity();
    leaderAppliedVolts = leaderMotor.getMotorVoltage();
    leaderCurrent = leaderMotor.getStatorCurrent();

    follower1Velocity = followerMotor1.getVelocity();
    follower1AppliedVolts = followerMotor1.getMotorVoltage();
    follower1Current = followerMotor1.getStatorCurrent();

    follower2Velocity = followerMotor2.getVelocity();
    follower2AppliedVolts = followerMotor2.getMotorVoltage();
    follower2Current = followerMotor2.getStatorCurrent();

    follower3Velocity = followerMotor3.getVelocity();
    follower3AppliedVolts = followerMotor3.getMotorVoltage();
    follower3Current = followerMotor3.getStatorCurrent();

    // Set update frequencies — on CANivore (CAN FD), use higher rates
    BaseStatusSignal.setUpdateFrequencyForAll(
        200,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        follower1Velocity,
        follower1AppliedVolts,
        follower1Current,
        follower2Velocity,
        follower2AppliedVolts,
        follower2Current,
        follower3Velocity,
        follower3AppliedVolts,
        follower3Current);

    // Optimize CAN bus utilization
    ParentDevice.optimizeBusUtilizationForAll(
        leaderMotor, followerMotor1, followerMotor2, followerMotor3);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Leader
    StatusCode leaderStatus =
        BaseStatusSignal.refreshAll(leaderVelocity, leaderAppliedVolts, leaderCurrent);
    inputs.leaderMotorConnected = leaderDebouncer.calculate(leaderStatus.isOK());
    inputs.leaderMotorVelocityRPM = leaderVelocity.getValueAsDouble() * 60.0;
    inputs.leaderMotorVoltage = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderMotorCurrent = leaderCurrent.getValueAsDouble();

    // Follower 1
    StatusCode f1Status =
        BaseStatusSignal.refreshAll(follower1Velocity, follower1AppliedVolts, follower1Current);
    inputs.followerMotor1Connected = follower1Debouncer.calculate(f1Status.isOK());
    inputs.followerMotor1VelocityRPM = follower1Velocity.getValueAsDouble() * 60.0;
    inputs.followerMotor1Voltage = follower1AppliedVolts.getValueAsDouble();
    inputs.followerMotor1Current = follower1Current.getValueAsDouble();

    // Follower 2
    StatusCode f2Status =
        BaseStatusSignal.refreshAll(follower2Velocity, follower2AppliedVolts, follower2Current);
    inputs.followerMotor2Connected = follower2Debouncer.calculate(f2Status.isOK());
    inputs.followerMotor2VelocityRPM = follower2Velocity.getValueAsDouble() * 60.0;
    inputs.followerMotor2Voltage = follower2AppliedVolts.getValueAsDouble();
    inputs.followerMotor2Current = follower2Current.getValueAsDouble();

    // Follower 3
    StatusCode f3Status =
        BaseStatusSignal.refreshAll(follower3Velocity, follower3AppliedVolts, follower3Current);
    inputs.followerMotor3Connected = follower3Debouncer.calculate(f3Status.isOK());
    inputs.followerMotor3VelocityRPM = follower3Velocity.getValueAsDouble() * 60.0;
    inputs.followerMotor3Voltage = follower3AppliedVolts.getValueAsDouble();
    inputs.followerMotor3Current = follower3Current.getValueAsDouble();

    inputs.shooterSetpointRPM = setpointRPM;
  }

  @Override
  public void setVoltage(double voltage) {
    leaderMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public double getShooterRPM() {
    return leaderVelocity.getValueAsDouble() * 60.0;
  }

  @Override
  public double getVelocity() {
    return leaderVelocity.getValueAsDouble() * 60.0;
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
  public void setShooterRPM(double rpm) {
    this.setpointRPM = rpm;
    // Convert RPM → rotations per second for TalonFX
    double rps = rpm / 60.0;
    leaderMotor.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public boolean isAtSetpoint() {
    return Math.abs(setpointRPM - getShooterRPM()) <= ShooterConstants.kTolerance;
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
    leaderMotor.getConfigurator().apply(slot0);
  }

  @Override
  public void resetEncoder() {
    tryUntilOk(5, () -> leaderMotor.setPosition(0.0, 0.25));
  }

  @Override
  public void setBrakeMode(boolean brake) {
    var mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    leaderMotor.setNeutralMode(mode);
    followerMotor1.setNeutralMode(mode);
    followerMotor2.setNeutralMode(mode);
    followerMotor3.setNeutralMode(mode);
  }
}
