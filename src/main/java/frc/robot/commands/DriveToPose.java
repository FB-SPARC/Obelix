// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.geometry.GeomUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Drives to a target field pose using a trapezoidal motion profile for linear motion and a profiled
 * PID controller for heading. All gains are backed by {@link LoggedTunableNumber} so they can be
 * adjusted live in tuning mode.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * new DriveToPose(drive, () -> targetPose)
 * new DriveToPose(drive, () -> targetPose, () -> Optional.of(omegaRad))  // heading override
 * }</pre>
 */
public class DriveToPose extends Command {

  // ── Tunable gains ─────────────────────────────────────────────────────────────
  private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("DriveToPose/DrivekP");
  private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("DriveToPose/DrivekD");
  private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("DriveToPose/ThetakP");
  private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("DriveToPose/ThetakD");
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration");
  private static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity");
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveToPose/DriveTolerance");
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("DriveToPose/ThetaTolerance");
  private static final LoggedTunableNumber linearFFMinRadius =
      new LoggedTunableNumber("DriveToPose/LinearFFMinRadius");
  private static final LoggedTunableNumber linearFFMaxRadius =
      new LoggedTunableNumber("DriveToPose/LinearFFMaxRadius");

  static {
    drivekP.initDefault(1.8);
    drivekD.initDefault(0.0);
    thetakP.initDefault(5.0);
    thetakD.initDefault(0.5);

    driveMaxVelocity.initDefault(4.0); // m/s
    driveMaxAcceleration.initDefault(4.0); // m/s²

    thetaMaxVelocity.initDefault(Units.degreesToRadians(500.0)); // rad/s
    thetaMaxAcceleration.initDefault(8.0); // rad/s²

    driveTolerance.initDefault(0.01); // meters
    thetaTolerance.initDefault(Units.degreesToRadians(1.0)); // radians

    linearFFMinRadius.initDefault(0.01); // start scaling FF at this distance
    linearFFMaxRadius.initDefault(0.05); // full FF beyond this distance
  }

  // ── Fields ────────────────────────────────────────────────────────────────────
  private final Drive drive;
  private final Supplier<Pose2d> target;

  private TrapezoidProfile driveProfile;
  private final PIDController driveController =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);

  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Rotation2d lastGoalRotation = Rotation2d.kZero;
  private double lastTime = 0.0;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private boolean running = false;

  public DriveToPose(Drive drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = target.get();

    driveProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));

    driveController.reset();
    thetaController.reset(currentPose.getRotation().getRadians(), 0.0);

    lastSetpointTranslation = currentPose.getTranslation();
    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();
  }

  @Override
  public void execute() {
    running = true;

    // ── Update gains from tunable numbers ─────────────────────────
    if (drivekP.hasChanged(hashCode())
        || drivekD.hasChanged(hashCode())
        || thetakP.hasChanged(hashCode())
        || thetakD.hasChanged(hashCode())
        || driveTolerance.hasChanged(hashCode())
        || thetaTolerance.hasChanged(hashCode())) {
      driveController.setP(drivekP.get());
      driveController.setD(drivekD.get());
      driveController.setTolerance(driveTolerance.get());
      thetaController.setP(thetakP.get());
      thetaController.setD(thetakD.get());
      thetaController.setTolerance(thetaTolerance.get());
    }

    driveProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));

    // ── Pose error ────────────────────────────────────────────────
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = target.get();

    Pose2d poseError = currentPose.relativeTo(targetPose);
    driveErrorAbs = poseError.getTranslation().getNorm();
    thetaErrorAbs = Math.abs(poseError.getRotation().getRadians());

    double linearFFScaler =
        MathUtil.clamp(
            (driveErrorAbs - linearFFMinRadius.get())
                / (linearFFMaxRadius.get() - linearFFMinRadius.get()),
            0.0,
            1.0);

    // ── Linear motion via trapezoid profile ───────────────────────
    Rotation2d targetToCurrentAngle =
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();

    State driveSetpoint =
        driveProfile.calculate(
            Constants.loopPeriodSecs,
            new State(targetPose.getTranslation().getDistance(lastSetpointTranslation), 0.0),
            new State(0.0, 0.0));

    double driveVelocityScalar =
        driveController.calculate(driveErrorAbs, driveSetpoint.position)
            + driveSetpoint.velocity * linearFFScaler;
    if (driveErrorAbs < driveController.getErrorTolerance()) driveVelocityScalar = 0.0;

    Translation2d driveVelocity = new Translation2d(driveVelocityScalar, targetToCurrentAngle);
    lastSetpointTranslation =
        new Pose2d(targetPose.getTranslation(), targetToCurrentAngle)
            .transformBy(GeomUtil.toTransform2d(driveSetpoint.position, 0.0))
            .getTranslation();

    // ── Theta control ─────────────────────────────────────────────
    double thetaSetpointVelocity =
        Math.abs(targetPose.getRotation().minus(lastGoalRotation).getDegrees()) < 10.0
            ? targetPose.getRotation().minus(lastGoalRotation).getRadians()
                / (Timer.getTimestamp() - lastTime)
            : thetaController.getSetpoint().velocity;

    double thetaVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(),
                new State(targetPose.getRotation().getRadians(), thetaSetpointVelocity))
            + thetaController.getSetpoint().velocity;
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();

    // ── Command drive ─────────────────────────────────────────────
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    // ── Logging ───────────────────────────────────────────────────
    Logger.recordOutput("DriveToPose/DistanceMeasured", driveErrorAbs);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveSetpoint.position);
    Logger.recordOutput("DriveToPose/ThetaMeasuredRad", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpointRad", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d[] {
          new Pose2d(
              lastSetpointTranslation,
              Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    running = false;
    Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
  }

  /**
   * Returns true when the robot is within the specified tolerances of the goal pose.
   *
   * @param driveToleranceMeters linear tolerance in meters
   * @param thetaTolerance heading tolerance
   */
  public boolean withinTolerance(double driveToleranceMeters, Rotation2d thetaTolerance) {
    return running
        && driveErrorAbs < driveToleranceMeters
        && thetaErrorAbs < thetaTolerance.getRadians();
  }

  /** Returns whether this command is actively running (not yet ended). */
  public boolean isRunning() {
    return running;
  }
}
