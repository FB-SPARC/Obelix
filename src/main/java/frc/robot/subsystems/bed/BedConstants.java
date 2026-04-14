// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.bed;

import frc.robot.util.energy.CurrentLimits;

public final class BedConstants {
  public static final int LEADER_MOTOR_ID = 19;
  public static final int FOLLOWER_MOTOR_ID = 20;

  public static final double MAX_CURRENT = CurrentLimits.kBed;

  public static final double kTolerance = 50; // RPM

  public static final double kP = 0.3;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.2;
  public static final double kV = 0.1;
  public static final double kA = 0.0;
}
