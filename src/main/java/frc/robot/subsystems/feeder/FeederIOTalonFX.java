// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.feeder;

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
 * FeederIOTalonFX implements the FeederIO interface using two CTRE TalonFX motor controllers. The
 * leader drives velocity closed-loop control and the follower mirrors the leader output.
 */
public class FeederIOTalonFX implements FeederIO {

  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true);

  // Leader status signals
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;
  private final StatusSignal<Current> leaderSupplyCurrent;

  // Follower status signals
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;

  // Connection debouncers
  private final Debouncer leaderConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5);

  public FeederIOTalonFX() {
    leaderMotor = new TalonFX(FeederConstants.LEADER_MOTOR_ID, Constants.canivore);
    followerMotor = new TalonFX(FeederConstants.FOLLOWER_MOTOR_ID, Constants.canivore);

    // Leader configuration
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = FeederConstants.MAX_CURRENT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // PID + feedforward for velocity control
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = FeederConstants.kP;
    slot0.kI = FeederConstants.kI;
    slot0.kD = FeederConstants.kD;
    slot0.kS = FeederConstants.kS;
    slot0.kV = FeederConstants.kV;
    slot0.kA = FeederConstants.kA;

    // Apply configuration to leader
    tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> leaderMotor.setPosition(0.0, 0.25));

    // Follower configuration
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerConfig.CurrentLimits.StatorCurrentLimit = FeederConstants.MAX_CURRENT;
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    tryUntilOk(5, () -> followerMotor.getConfigurator().apply(followerConfig, 0.25));

    // Configure follower to follow leader
    followerMotor.setControl(
        new Follower(FeederConstants.LEADER_MOTOR_ID, MotorAlignmentValue.Opposed));

    // Initialize leader status signals
    leaderVelocity = leaderMotor.getVelocity();
    leaderAppliedVolts = leaderMotor.getMotorVoltage();
    leaderCurrent = leaderMotor.getStatorCurrent();
    leaderSupplyCurrent = leaderMotor.getSupplyCurrent();

    // Initialize follower status signals
    followerVelocity = followerMotor.getVelocity();
    followerAppliedVolts = followerMotor.getMotorVoltage();
    followerCurrent = followerMotor.getStatorCurrent();
    followerSupplyCurrent = followerMotor.getSupplyCurrent();

    // Set update frequencies — on CANivore (CAN FD), use higher rates
    BaseStatusSignal.setUpdateFrequencyForAll(
        200,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        leaderSupplyCurrent,
        followerVelocity,
        followerAppliedVolts,
        followerCurrent,
        followerSupplyCurrent);

    // Optimize CAN bus utilization
    ParentDevice.optimizeBusUtilizationForAll(leaderMotor, followerMotor);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    StatusCode leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderVelocity, leaderAppliedVolts, leaderCurrent, leaderSupplyCurrent);
    StatusCode followerStatus =
        BaseStatusSignal.refreshAll(
            followerVelocity, followerAppliedVolts, followerCurrent, followerSupplyCurrent);

    inputs.leaderMotorConnected = leaderConnectedDebouncer.calculate(leaderStatus.isOK());
    inputs.leaderMotorVelocityRPM = leaderVelocity.getValueAsDouble() * 60.0;
    inputs.leaderMotorVoltage = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderMotorCurrent = leaderCurrent.getValueAsDouble();
    inputs.leaderMotorSupplyCurrent = leaderSupplyCurrent.getValueAsDouble();

    inputs.followerMotorConnected = followerConnectedDebouncer.calculate(followerStatus.isOK());
    inputs.followerMotorVelocityRPM = followerVelocity.getValueAsDouble() * 60.0;
    inputs.followerMotorVoltage = followerAppliedVolts.getValueAsDouble();
    inputs.followerMotorCurrent = followerCurrent.getValueAsDouble();
    inputs.followerMotorSupplyCurrent = followerSupplyCurrent.getValueAsDouble();
  }

  @Override
  public void applyOutputs(FeederIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> leaderMotor.setControl(voltageRequest.withOutput(0.0));
      case VOLTAGE -> leaderMotor.setControl(voltageRequest.withOutput(outputs.volts));
      case VELOCITY -> leaderMotor.setControl(
          velocityRequest.withVelocity(outputs.velocityRPM / 60.0));
    }
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {
    var slot0 = new com.ctre.phoenix6.configs.Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    leaderMotor.getConfigurator().apply(slot0);
  }
}
