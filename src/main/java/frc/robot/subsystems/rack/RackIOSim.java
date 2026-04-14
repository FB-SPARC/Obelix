package frc.robot.subsystems.rack;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

/** Simulated rack (linear elevator) IO using a WPILib ElevatorSim (1x Kraken X60). */
public class RackIOSim implements RackIO {
  private static final DCMotor kMotor = DCMotor.getKrakenX60(1);
  private static final double kCarriageMassKg = 2.0;

  private final ElevatorSim sim =
      new ElevatorSim(
          kMotor,
          RackConstants.kGearRatio,
          kCarriageMassKg,
          RackConstants.kPinionRadiusMeters,
          RackConstants.MIN_POSITION_METERS,
          RackConstants.MAX_POSITION_METERS,
          true,
          RackConstants.MIN_POSITION_METERS);

  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          RackConstants.kP,
          RackConstants.kI,
          RackConstants.kD,
          new TrapezoidProfile.Constraints(
              RackConstants.kPinionRadiusMeters
                  * Units.rotationsPerMinuteToRadiansPerSecond(
                      RackConstants.kCruiseVelocity * 60.0),
              RackConstants.kPinionRadiusMeters
                  * Units.rotationsPerMinuteToRadiansPerSecond(
                      RackConstants.kAcceleration * 60.0)));

  private final ElevatorFeedforward ff =
      new ElevatorFeedforward(RackConstants.kS, 0.0, RackConstants.kV, RackConstants.kA);

  private double appliedVoltage = 0.0;

  @Override
  public void updateInputs(RackIOInputs inputs) {
    sim.update(Constants.loopPeriodSecs);

    double motorRotations =
        sim.getPositionMeters()
            / (2 * Math.PI * RackConstants.kPinionRadiusMeters)
            * RackConstants.kGearRatio;

    inputs.motorConnected = true;
    inputs.motorPositionDegrees = motorRotations * 360.0;
    inputs.motorVelocityDegreesPerSecond =
        sim.getVelocityMetersPerSecond()
            / (2 * Math.PI * RackConstants.kPinionRadiusMeters)
            * RackConstants.kGearRatio
            * 360.0;
    inputs.motorVoltage = appliedVoltage;
    inputs.motorCurrent = sim.getCurrentDrawAmps();
    inputs.rackPositionMeters = sim.getPositionMeters();
    inputs.mechanismPositionMeters = sim.getPositionMeters();
  }

  @Override
  public void applyOutputs(RackIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> appliedVoltage = 0.0;
      case VOLTAGE -> appliedVoltage = outputs.volts;
      case POSITION -> appliedVoltage =
          pid.calculate(sim.getPositionMeters(), outputs.positionMeters)
              + ff.calculate(pid.getSetpoint().velocity);
    }
    appliedVoltage = Math.max(-12.0, Math.min(12.0, appliedVoltage));
    sim.setInputVoltage(appliedVoltage);
  }
}
