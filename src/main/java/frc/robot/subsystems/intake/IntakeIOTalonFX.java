// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
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
  private final StatusSignal<Current> leaderSupplyCurrent;

  // Follower status signals
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;

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
  public void updateInputs(IntakeIOInputs inputs) {
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
  public void applyOutputs(IntakeIOOutputs outputs) {
    if (Constants.tuningMode) {
      var neutralMode = outputs.brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
      leaderMotor.setNeutralMode(neutralMode);
      followerMotor.setNeutralMode(neutralMode);
    }
    switch (outputs.mode) {
      case BRAKE -> leaderMotor.setControl(voltageRequest.withOutput(0.0));
      case VOLTAGE -> leaderMotor.setControl(voltageRequest.withOutput(outputs.volts));
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
