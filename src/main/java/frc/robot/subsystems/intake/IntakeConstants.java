// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.intake;

import frc.robot.util.energy.CurrentLimits;

public final class IntakeConstants {
  public static final int MOTOR_ID = 17;
  public static final int FOLLOWER_MOTOR_ID = 2;

  public static final double MAX_CURRENT = CurrentLimits.kIntake;

  public static final double kTolerance = 50; // RPM

  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
}
