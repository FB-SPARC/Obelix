// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Simulated feeder IO using a WPILib DCMotorSim (2x Kraken X60, direct drive). */
public class FeederIOSim implements FeederIO {
  private static final DCMotor kMotor = DCMotor.getKrakenX60(2);
  private static final double kMOI = 0.001;
  private static final double kGearRatio = 1.0;

  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(kMotor, kMOI, kGearRatio), kMotor);

  private double appliedVoltage = 0.0;

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    sim.update(Constants.loopPeriodSecs);

    double velocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec());
    double current = sim.getCurrentDrawAmps();

    inputs.leaderMotorConnected = true;
    inputs.leaderMotorVelocityRPM = velocityRPM;
    inputs.leaderMotorVoltage = appliedVoltage;
    inputs.leaderMotorCurrent = current / 2.0;
    inputs.leaderMotorSupplyCurrent = (current / 2.0) * Math.abs(appliedVoltage) / 12.0;

    inputs.followerMotorConnected = true;
    inputs.followerMotorVelocityRPM = velocityRPM;
    inputs.followerMotorVoltage = appliedVoltage;
    inputs.followerMotorCurrent = current / 2.0;
    inputs.followerMotorSupplyCurrent = (current / 2.0) * Math.abs(appliedVoltage) / 12.0;
  }

  @Override
  public void applyOutputs(FeederIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> appliedVoltage = 0.0;
      case VOLTAGE -> appliedVoltage = outputs.volts;
      case VELOCITY -> appliedVoltage = outputs.velocityRPM * FeederConstants.kV;
    }
    appliedVoltage = Math.max(-12.0, Math.min(12.0, appliedVoltage));
    sim.setInputVoltage(appliedVoltage);
  }
}
