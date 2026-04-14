// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public boolean leaderMotorConnected = false;
    public double leaderMotorCurrent = 0.0;
    public double leaderMotorSupplyCurrent = 0.0;
    public double leaderMotorVoltage = 0.0;
    public double leaderMotorVelocityRPM = 0.0;

    public boolean followerMotorConnected = false;
    public double followerMotorCurrent = 0.0;
    public double followerMotorSupplyCurrent = 0.0;
    public double followerMotorVoltage = 0.0;
    public double followerMotorVelocityRPM = 0.0;
  }

  public static enum FeederOutputMode {
    VOLTAGE,
    VELOCITY,
    BRAKE
  }

  public static class FeederIOOutputs {
    public FeederOutputMode mode = FeederOutputMode.BRAKE;
    public double velocityRPM = 0.0;
    public double volts = 0.0;
    public boolean brakeMode = false;
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void applyOutputs(FeederIOOutputs outputs) {}

  /** Applies updated PID/FF gains to the motor controller (no-op in sim). */
  public default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
