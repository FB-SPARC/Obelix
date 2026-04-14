// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.rack;

import frc.robot.util.energy.CurrentLimits;

public final class RackConstants {
  public static final int MOTOR_ID = 15;

  public static final double kGearRatio = 169.0 / 48.0;
  public static final double kPinionRadiusMeters = 0.02032;

  public static final double MAX_CURRENT = CurrentLimits.kRack;

  public static final double MAX_POSITION_METERS = 0.285;
  public static final double MIN_POSITION_METERS = 0.0;

  public static final double kTolerance = 0.05; // Meters

  // PID + feedforward gains
  public static final double kP = 7.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  // Motion Magic profile
  public static final double kCruiseVelocity = 5.0; // rot/s
  public static final double kAcceleration = 15.0; // rot/s^2
  public static final double kJerk = 800.0; // rot/s^3
}
