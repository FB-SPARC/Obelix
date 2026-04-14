// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.bed;

import org.littletonrobotics.junction.AutoLog;

public interface BedIO {
  @AutoLog
  public static class BedIOInputs {
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

  public static enum BedOutputMode {
    VOLTAGE,
    VELOCITY,
    BRAKE
  }

  public static class BedIOOutputs {
    public BedOutputMode mode = BedOutputMode.BRAKE;
    public double velocityRPM = 0.0;
    public double volts = 0.0;
    public boolean brakeMode = false;
  }

  public default void updateInputs(BedIOInputs inputs) {}

  public default void applyOutputs(BedIOOutputs outputs) {}
}
