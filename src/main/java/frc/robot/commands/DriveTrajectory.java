// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.commands;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Follows a Choreo trajectory using feedforward + PID control. */
public class DriveTrajectory extends Command {
  private static final LoggedTunableNumber linearkP =
      new LoggedTunableNumber("DriveTrajectory/LinearkP", 8.0);
  private static final LoggedTunableNumber linearkD =
      new LoggedTunableNumber("DriveTrajectory/LinearkD", 0.0);
  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("DriveTrajectory/ThetakP", 4.0);
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("DriveTrajectory/ThetakD", 0.0);

  private final Timer timer = new Timer();
  private final Trajectory<SwerveSample> trajectory;
  private final Drive drive;
  private final BooleanSupplier mirror;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  public DriveTrajectory(Trajectory<SwerveSample> trajectory, Drive drive, BooleanSupplier mirror) {
    this.trajectory = trajectory;
    this.drive = drive;
    this.mirror = mirror;
    xController = new PIDController(linearkP.get(), 0, linearkD.get(), Constants.loopPeriodSecs);
    yController = new PIDController(linearkP.get(), 0, linearkD.get(), Constants.loopPeriodSecs);
    thetaController = new PIDController(thetakP.get(), 0, thetakD.get(), Constants.loopPeriodSecs);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    timer.restart();
    Pose2d[] poses =
        trajectory.samples().stream().map(SwerveSample::getPose).toArray(Pose2d[]::new);
    Logger.recordOutput("Odometry/Trajectory", poses);
  }

  @Override
  public void execute() {
    if (linearkP.hasChanged(hashCode()) || linearkD.hasChanged(hashCode())) {
      xController.setPID(linearkP.get(), 0, linearkD.get());
      yController.setPID(linearkP.get(), 0, linearkD.get());
    }
    if (thetakP.hasChanged(hashCode()) || thetakD.hasChanged(hashCode())) {
      thetaController.setPID(thetakP.get(), 0, thetakD.get());
    }

    var sampleOpt = trajectory.sampleAt(timer.get(), mirror.getAsBoolean());
    if (sampleOpt.isEmpty()) return;
    SwerveSample sample = sampleOpt.get();

    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Pose2d targetPose = sample.getPose();

    double xOutput = xController.calculate(currentPose.getX(), targetPose.getX()) + sample.vx;
    double yOutput = yController.calculate(currentPose.getY(), targetPose.getY()) + sample.vy;
    double thetaOutput =
        thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians())
            + sample.omega;

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(xOutput, yOutput, thetaOutput), currentPose.getRotation()));

    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTime());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    Logger.recordOutput("Odometry/Trajectory", new Pose2d[] {});
    Logger.recordOutput("Odometry/TrajectorySetpoint", Pose2d.kZero);
  }
}
