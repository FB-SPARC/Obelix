package frc.robot.subsystems.bed;

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
 * BedIOTalonFX implements the BedIO interface using two CTRE TalonFX motor controllers. The leader
 * drives velocity closed-loop control and the follower mirrors the leader output.
 */
public class BedIOTalonFX implements BedIO {

  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true);

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

  public BedIOTalonFX() {
    leaderMotor = new TalonFX(BedConstants.LEADER_MOTOR_ID);
    followerMotor = new TalonFX(BedConstants.FOLLOWER_MOTOR_ID);

    // Leader configuration
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = BedConstants.MAX_CURRENT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // PID + feedforward for velocity control
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = BedConstants.kP;
    slot0.kI = BedConstants.kI;
    slot0.kD = BedConstants.kD;
    slot0.kS = BedConstants.kS;
    slot0.kV = BedConstants.kV;
    slot0.kA = BedConstants.kA;

    // Apply configuration to leader
    tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> leaderMotor.setPosition(0.0, 0.25));

    // Configure follower to follow leader
    followerMotor.setControl(
        new Follower(BedConstants.LEADER_MOTOR_ID, MotorAlignmentValue.Opposed));

    // Initialize leader status signals
    leaderVelocity = leaderMotor.getVelocity();
    leaderAppliedVolts = leaderMotor.getMotorVoltage();
    leaderCurrent = leaderMotor.getStatorCurrent();

    // Initialize follower status signals
    followerVelocity = followerMotor.getVelocity();
    followerAppliedVolts = followerMotor.getMotorVoltage();
    followerCurrent = followerMotor.getStatorCurrent();

    // Set update frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
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
  public void updateInputs(BedIOInputs inputs) {
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
  public void applyOutputs(BedIOOutputs outputs) {
    if (Constants.tuningMode) {
      var neutralMode = outputs.brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
      leaderMotor.setNeutralMode(neutralMode);
      followerMotor.setNeutralMode(neutralMode);
    }
    switch (outputs.mode) {
      case BRAKE -> leaderMotor.setControl(voltageRequest.withOutput(0.0));
      case VOLTAGE -> leaderMotor.setControl(voltageRequest.withOutput(outputs.volts));
      case VELOCITY -> leaderMotor.setControl(
          velocityRequest.withVelocity(outputs.velocityRPM / 60.0));
    }
  }
}
