// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.util.energy;

/**
 * Single source of truth for all TalonFX stator current limits (amps).
 *
 * <p>Change a limit here to affect both the motor configuration and the BatteryLogger's overhead
 * calculations without touching subsystem-specific files.
 */
public final class CurrentLimits {
  private CurrentLimits() {}

  /** Stator current limit for each Bed roller motor (leader + follower). */
  public static final double kBed = 100.0;

  /** Stator current limit for each Feeder roller motor (leader + follower). */
  public static final double kFeeder = 100.0;

  /** Stator current limit for the Hood motor. */
  public static final double kHood = 100.0;

  /** Stator current limit for each Intake motor (leader + follower). */
  public static final double kIntake = 60.0;

  /** Stator current limit for the Rack motor. */
  public static final double kRack = 55.0;

  /** Stator current limit for each Shooter motor (leader + 3 followers). */
  public static final double kShooter = 100.0;
}
