// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leaderMotorConnected = false;
    public double leaderMotorCurrent = 0.0;
    public double leaderMotorSupplyCurrent = 0.0;
    public double leaderMotorVoltage = 0.0;
    public double leaderMotorVelocityRPM = 0.0;

    public boolean followerMotor1Connected = false;
    public double followerMotor1Current = 0.0;
    public double followerMotor1SupplyCurrent = 0.0;
    public double followerMotor1Voltage = 0.0;
    public double followerMotor1VelocityRPM = 0.0;

    public boolean followerMotor2Connected = false;
    public double followerMotor2Current = 0.0;
    public double followerMotor2SupplyCurrent = 0.0;
    public double followerMotor2Voltage = 0.0;
    public double followerMotor2VelocityRPM = 0.0;

    public boolean followerMotor3Connected = false;
    public double followerMotor3Current = 0.0;
    public double followerMotor3SupplyCurrent = 0.0;
    public double followerMotor3Voltage = 0.0;
    public double followerMotor3VelocityRPM = 0.0;
  }

  public static enum ShooterOutputMode {
    VOLTAGE,
    VELOCITY,
    BRAKE
  }

  public static class ShooterIOOutputs {
    public ShooterOutputMode mode = ShooterOutputMode.BRAKE;
    public double velocityRPM = 0.0;
    public double volts = 0.0;
    public boolean brakeMode = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void applyOutputs(ShooterIOOutputs outputs) {}

  /** Applies updated PID/FF gains to the motor controller (no-op in sim). */
  public default void setPID(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
