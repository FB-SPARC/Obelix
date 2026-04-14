// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.hood;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/**
 * Simulated hood IO using a WPILib SingleJointedArmSim (1x Kraken X60). Outputs are applied
 * atomically via applyOutputs().
 */
public class HoodIOSim implements HoodIO {
  private static final DCMotor kMotor = DCMotor.getKrakenX60(1);
  private static final double kArmLengthMeters = 0.25;
  private static final double kArmMassKg = 0.5;

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          kMotor,
          HoodConstants.kGearRatio * HoodConstants.kSensorToMechanismRatio,
          SingleJointedArmSim.estimateMOI(kArmLengthMeters, kArmMassKg),
          kArmLengthMeters,
          Units.degreesToRadians(HoodConstants.MIN_ANGLE_DEGREES),
          Units.degreesToRadians(HoodConstants.MAX_ANGLE_DEGREES),
          true,
          Units.degreesToRadians(HoodConstants.MIN_ANGLE_DEGREES));

  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          0.03,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              Units.degreesToRadians(HoodConstants.kCruiseVelocity * 360.0),
              Units.degreesToRadians(HoodConstants.kAcceleration * 360.0)));

  private double appliedVoltage = 0.0;

  public HoodIOSim() {
    pid.setTolerance(Units.degreesToRadians(HoodConstants.kTolerance));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    sim.update(Constants.loopPeriodSecs);

    inputs.motorConnected = true;
    inputs.motorPositionDegrees = Units.radiansToDegrees(sim.getAngleRads());
    inputs.motorVelocityDegreesPerSecond = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    inputs.motorVoltage = appliedVoltage;
    inputs.motorCurrent = sim.getCurrentDrawAmps();
    inputs.motorSupplyCurrent = sim.getCurrentDrawAmps() * Math.abs(appliedVoltage) / 12.0;
    inputs.mechanismPositionDegrees = Units.radiansToDegrees(sim.getAngleRads());
    inputs.encoderConnected = true;
    inputs.absoluteEncoderPositionDegrees = Units.radiansToDegrees(sim.getAngleRads());
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> appliedVoltage = 0.0;
      case VOLTAGE -> appliedVoltage = outputs.volts;
      case POSITION -> appliedVoltage =
          pid.calculate(sim.getAngleRads(), Units.degreesToRadians(outputs.positionDegrees));
    }
    appliedVoltage = Math.max(-12.0, Math.min(12.0, appliedVoltage));
    sim.setInputVoltage(appliedVoltage);
  }
}
