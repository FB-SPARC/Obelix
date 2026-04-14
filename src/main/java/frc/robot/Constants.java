// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /** Main robot loop period in seconds. */
  public static final double loopPeriodSecs = 0.02;

  /**
   * Watchdog timeout in seconds. Longer than loopPeriodSecs to suppress spurious DS loop overrun
   * warnings on heavy initialization frames while still catching genuinely broken loops.
   */
  public static final double loopPeriodWatchdogSecs = 0.2;

  /**
   * If true, PID/FF gains are exposed on NetworkTables and can be changed without a redeploy. Set
   * to false for competition.
   */
  public static final boolean tuningMode = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** The physical robot being used. Set to SIMBOT when running in simulation. */
  private static final RobotType robot = RobotType.OBELIX;

  public static RobotType getRobot() {
    boolean isDesktop = !RobotBase.isReal();
    return isDesktop && System.getenv("SIMBOT") != null ? RobotType.SIMBOT : robot;
  }

  public enum RobotType {
    OBELIX,
    SIMBOT
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robot == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robot);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robot != RobotType.OBELIX || tuningMode) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }

  /** Disable HAL (Hardware Abstraction Layer) for testing purposes. */
  public static boolean disableHAL;

  public static CANBus canivore = new CANBus("Canivore");

  public static void disableHAL() {
    disableHAL = true;
  }

  public static final class DriveAlignConstants {
    public static final double ANGLE_KP = 5.0;
    public static final double ANGLE_KD = 0.0;
    public static final double ANGLE_MAX_VELOCITY = 8.0;
    public static final double ANGLE_MAX_ACCELERATION = 20.0;
  }
}
