// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.rack;

import org.littletonrobotics.junction.AutoLog;

public interface RackIO {
  @AutoLog
  public static class RackIOInputs {
    public boolean motorConnected = false;
    public double motorCurrent = 0.0;
    public double motorSupplyCurrent = 0.0;
    public double motorVoltage = 0.0;
    public double motorPositionDegrees = 0.0;
    public double motorVelocityDegreesPerSecond = 0.0;
    public double rackPositionMeters = 0.0;
    public double mechanismPositionMeters = 0.0;
  }

  public static enum RackOutputMode {
    VOLTAGE,
    POSITION,
    BRAKE
  }

  public static class RackIOOutputs {
    public RackOutputMode mode = RackOutputMode.BRAKE;
    public double positionMeters = 0.0;
    public double volts = 0.0;
    public boolean brakeMode = true;
    // Motion Magic / feedforward overrides (optional, passed from Superstructure)
    public double kv = 0.0;
    public double ka = 0.0;
    public double kj = 0.0;
  }

  public default void updateInputs(RackIOInputs inputs) {}

  public default void applyOutputs(RackIOOutputs outputs) {}

  /** Applies updated PID/FF gains to the motor controller (no-op in sim). */
  public default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
