// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.hood;

import frc.robot.util.energy.CurrentLimits;

public final class HoodConstants {
  public static final int MOTOR_ID = 60;
  public static final int CANCODER_ID = 40;

  public static final double kGearRatio = 4.0; // Motor-to-CANCoder: 4:1 reduction
  public static final double kSensorToMechanismRatio =
      17.6; // CANCoder rotations to hood output rotations

  public static final double MAX_CURRENT = CurrentLimits.kHood;

  public static final double MAX_ANGLE_DEGREES = 22.5;
  public static final double MIN_ANGLE_DEGREES = 0;

  public static final double kEncoderOffset =
      44.297; // 41.045; // 79.014; // CANCoder magnet offset in degrees

  public static final double kTolerance = 0.5; // Degrees

  // PID + feedforward gains
  public static final double kP = 550;
  public static final double kI = 250;
  public static final double kD = 7.5;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  // Motion Magic profile
  public static final double kCruiseVelocity = 40.0; // rot/s
  public static final double kAcceleration = 80.0; // rot/s^2
  public static final double kJerk = 800.0; // rot/s^3
}
