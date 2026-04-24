// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
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
  private final StatusSignal<Current> supplyCurrent;

  // Connection debouncer
  private final Debouncer connectedDebouncer = new Debouncer(0.5);

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
    supplyCurrent = motor.getSupplyCurrent();

    // Set update frequencies — on CANivore (CAN FD), use higher rates; rack moves fast
    BaseStatusSignal.setUpdateFrequencyForAll(
        200, position, velocity, appliedVolts, current, supplyCurrent);

    // Optimize CAN bus utilization
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(RackIOInputs inputs) {
    StatusCode status =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current, supplyCurrent);

    inputs.motorConnected = connectedDebouncer.calculate(status.isOK());
    inputs.motorPositionDegrees =
        Units.rotationsToDegrees(position.getValueAsDouble()) / RackConstants.kGearRatio;
    inputs.motorVelocityDegreesPerSecond =
        Units.rotationsToDegrees(velocity.getValueAsDouble()) / RackConstants.kGearRatio;
    inputs.rackPositionMeters = motorRotationsToMeters(position.getValueAsDouble());
    inputs.mechanismPositionMeters = motorRotationsToMeters(position.getValueAsDouble());
    inputs.motorVoltage = appliedVolts.getValueAsDouble();
    inputs.motorCurrent = current.getValueAsDouble();
    inputs.motorSupplyCurrent = supplyCurrent.getValueAsDouble();
  }

  @Override
  public void applyOutputs(RackIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> motor.setControl(voltageRequest.withOutput(0.0));
      case VOLTAGE -> motor.setControl(voltageRequest.withOutput(outputs.volts));
      case POSITION -> motor.setControl(
          motionMagicRequest
              .withPosition(metersToMotorRotations(outputs.positionMeters))
              .withVelocity(outputs.kv)
              .withAcceleration(outputs.ka)
              .withJerk(outputs.kj));
    }
  }

  /** Converts linear meters to motor rotations (accounting for gear ratio and pinion). */
  private double metersToMotorRotations(double meters) {
    double mechanismRotations = meters / kPinionCircumferenceMeters;
    return mechanismRotations * RackConstants.kGearRatio;
  }

  /** Converts motor rotations to linear meters (accounting for gear ratio and pinion). */
  private double motorRotationsToMeters(double motorRotations) {
    double mechanismRotations = motorRotations / RackConstants.kGearRatio;
    return mechanismRotations * kPinionCircumferenceMeters;
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
    motor.getConfigurator().apply(slot0);
  }
}
