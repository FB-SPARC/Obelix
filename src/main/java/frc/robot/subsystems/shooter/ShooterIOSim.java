package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Simulated shooter IO using a WPILib DCMotorSim (4x Kraken X60, direct drive). */
public class ShooterIOSim implements ShooterIO {
  private static final DCMotor kMotor = DCMotor.getKrakenX60(4);
  private static final double kMOI = 0.004;
  private static final double kGearRatio = 1.0;

  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(kMotor, kMOI, kGearRatio), kMotor);

  private double appliedVoltage = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    sim.update(Constants.loopPeriodSecs);

    double velocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec());
    double current = sim.getCurrentDrawAmps();

    inputs.leaderMotorConnected = true;
    inputs.leaderMotorVelocityRPM = velocityRPM;
    inputs.leaderMotorVoltage = appliedVoltage;
    inputs.leaderMotorCurrent = current / 4.0;

    inputs.followerMotor1Connected = true;
    inputs.followerMotor1VelocityRPM = velocityRPM;
    inputs.followerMotor1Voltage = appliedVoltage;
    inputs.followerMotor1Current = current / 4.0;

    inputs.followerMotor2Connected = true;
    inputs.followerMotor2VelocityRPM = velocityRPM;
    inputs.followerMotor2Voltage = appliedVoltage;
    inputs.followerMotor2Current = current / 4.0;

    inputs.followerMotor3Connected = true;
    inputs.followerMotor3VelocityRPM = velocityRPM;
    inputs.followerMotor3Voltage = appliedVoltage;
    inputs.followerMotor3Current = current / 4.0;
  }

  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> appliedVoltage = 0.0;
      case VOLTAGE -> appliedVoltage = outputs.volts;
      case VELOCITY -> {
        // Simple open-loop approximation using kV from constants
        double setpointRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(outputs.velocityRPM);
        appliedVoltage = setpointRadPerSec * ShooterConstants.kV;
      }
    }
    appliedVoltage = Math.max(-12.0, Math.min(12.0, appliedVoltage));
    sim.setInputVoltage(appliedVoltage);
  }
}
