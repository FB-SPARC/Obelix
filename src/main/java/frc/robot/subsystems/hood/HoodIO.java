// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean motorConnected = false;
    public double motorCurrent = 0.0;
    public double motorSupplyCurrent = 0.0;
    public double motorVoltage = 0.0;
    public double motorPositionDegrees = 0.0;
    public double motorVelocityDegreesPerSecond = 0.0;
    public double mechanismPositionDegrees = 0.0;

    public boolean encoderConnected = false;
    public double absoluteEncoderPositionDegrees = 0.0;
  }

  /** Output mode for the hood motor. */
  public static enum HoodOutputMode {
    VOLTAGE,
    POSITION,
    BRAKE
  }

  /** Desired outputs written by the subsystem and applied atomically by the IO layer. */
  public static class HoodIOOutputs {
    public HoodOutputMode mode = HoodOutputMode.BRAKE;
    /** Target position in degrees (used when mode == POSITION). */
    public double positionDegrees = 0.0;
    /** Target voltage (used when mode == VOLTAGE). */
    public double volts = 0.0;
    /** Whether to use brake or coast neutral mode. */
    public boolean brakeMode = true;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  /** Apply the desired outputs (called once per cycle after the scheduler). */
  public default void applyOutputs(HoodIOOutputs outputs) {}
}
