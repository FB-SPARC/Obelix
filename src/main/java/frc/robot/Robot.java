// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.FullSubsystem;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LoggedTracer;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.energy.BatteryLogger;
import java.lang.reflect.Field;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private double autoStart;
  private boolean autoMessagePrinted;
  private RobotContainer robotContainer;

  private static final Timer disabledTimer = new Timer();
  public static final BatteryLogger batteryLogger = new BatteryLogger();

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });
    try {
      Logger.recordMetadata("Hostname", InetAddress.getLocalHost().getHostName());
    } catch (UnknownHostException e) {
      Logger.recordMetadata("Hostname", "Unknown");
    }
    Logger.recordMetadata(
        "Platform", System.getProperty("os.name") + " " + System.getProperty("os.arch"));

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Adjust loop overrun warning timeout so spurious DS warnings don't fire
    // on heavy init frames while still catching genuinely broken loops
    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(Constants.loopPeriodWatchdogSecs);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to set watchdog timeout.", false);
    }
    CommandScheduler.getInstance().setPeriod(Constants.loopPeriodWatchdogSecs);

    // Silence joystick connection warnings in DS
    DriverStation.silenceJoystickConnectionWarning(true);

    // Silence "x and y components of Rotation2d are zero" spam
    var mathShared = MathSharedStore.getMathShared();
    MathSharedStore.setMathShared(
        new MathShared() {
          @Override
          public void reportError(String error, StackTraceElement[] stackTrace) {
            if (error.startsWith("x and y components of Rotation2d are zero")) {
              return;
            }
            mathShared.reportError(error, stackTrace);
          }

          @Override
          public void reportUsage(MathUsageId id, int count) {
            mathShared.reportUsage(id, count);
          }

          @Override
          public double getTimestamp() {
            return mathShared.getTimestamp();
          }
        });

    // Configure sim alliance station and team number
    if (Constants.currentMode == Constants.Mode.SIM) {
      RoboRioSim.setTeamNumber(5665);
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.notifyNewData();
    }

    // Reset disabled timer
    disabledTimer.restart();

    // Log active commands for replay debugging
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize(command -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt(command -> logCommandFunction.accept(command, false));

    // Instantiate our RobotContainer
    robotContainer = new RobotContainer();
  }

  /** Returns whether to display hardware fault alerts (suppressed for 30s after boot). */
  public static boolean showHardwareAlerts() {
    return Constants.currentMode != Constants.Mode.SIM && Timer.getTimestamp() > 30.0;
  }

  /** Returns whether performance should be throttled while disabled (after 5s idle). */
  public static boolean shouldThrottle() {
    return disabledTimer.hasElapsed(5.0);
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Reset tracer at the start of each loop cycle
    LoggedTracer.reset();

    // Update battery voltage for energy logging
    batteryLogger.setBatteryVoltage(edu.wpi.first.wpilibj.RobotController.getBatteryVoltage());
    batteryLogger.setRioCurrent(edu.wpi.first.wpilibj.RobotController.getInputCurrent());

    VirtualSubsystem.runAllPeriodic();
    CommandScheduler.getInstance().run();
    robotContainer.periodic();
    VirtualSubsystem.runAllPeriodicAfterScheduler();
    FullSubsystem.runAllPeriodicAfterScheduler();
    batteryLogger.periodicAfterScheduler();
    LoggedTracer.record("Robot/Scheduler");

    // Reset disabled timer while enabled
    if (DriverStation.isEnabled()) {
      disabledTimer.restart();
    }

    HubShiftUtil.ShiftInfo info = HubShiftUtil.getOfficialShiftInfo();
    Logger.recordOutput("Shift/Current", info.currentShift().toString());
    Logger.recordOutput("Shift/RemainingTime", info.remainingTime());
    Logger.recordOutput("Shift/ElapsedTime", info.elapsedTime());
    Logger.recordOutput("Shift/Active", info.active());

    // Print auto duration once the autonomous command finishes
    if (autonomousCommand != null) {
      if (!autonomousCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoStart = Timer.getTimestamp();
    autoMessagePrinted = false;
    HubShiftUtil.initialize();
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    HubShiftUtil.initialize();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
