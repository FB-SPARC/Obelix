// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.Drive;
import java.util.NoSuchElementException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

/**
 * Centralises robot state: pose estimation, velocity tracking, and timestamped pose lookup.
 *
 * <p>Design goals:
 *
 * <ul>
 *   <li>Any subsystem can obtain the current pose without holding a reference to {@link Drive}.
 *   <li>Field-relative velocity is available for shot-while-moving lead compensation.
 *   <li>A {@link TimeInterpolatableBuffer} allows callers to look up the estimated pose at a past
 *       camera timestamp, enabling proper vision latency compensation.
 * </ul>
 *
 * <p>Usage:
 *
 * <pre>{@code
 * // In Drive.periodic() — called for each odometry sample:
 * RobotState.getInstance().addOdometryObservation(timestamp, yaw, modulePositions);
 *
 * // In Vision.periodic() — instead of consumer.accept():
 * RobotState.getInstance().addVisionMeasurement(pose2d, timestamp, stdDevs);
 *
 * // Anywhere else:
 * Pose2d pose = RobotState.getInstance().getEstimatedPose();
 * }</pre>
 */
public class RobotState {

  // ── Constants ─────────────────────────────────────────────────────────────

  /** How many seconds of pose history to keep for latency-compensated vision fusion. */
  private static final double POSE_BUFFER_SIZE_SEC = 2.0;

  /** Odometry process-noise standard deviations (x m, y m, θ rad). */
  private static final Matrix<N3, N1> ODOMETRY_STD_DEVS = VecBuilder.fill(0.003, 0.003, 0.002);

  // ── Singleton ─────────────────────────────────────────────────────────────

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // ── State ──────────────────────────────────────────────────────────────────

  /** Raw odometry pose (no vision corrections). */
  @AutoLogOutput(key = "RobotState/OdometryPose")
  private Pose2d odometryPose = Pose2d.kZero;

  /** Vision-fused best estimate of the robot pose. */
  @AutoLogOutput(key = "RobotState/EstimatedPose")
  private Pose2d estimatedPose = Pose2d.kZero;

  /** Robot-relative measured chassis speeds — set each loop by Drive. */
  @AutoLogOutput(key = "RobotState/RobotVelocity")
  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  // History buffer for vision latency compensation
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE_SEC);

  // Kalman Q matrix (process noise covariance, squared std devs)
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  // Odometry internals
  private final SwerveDriveKinematics kinematics;
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d gyroOffset = Rotation2d.kZero;

  // ── Constructor ────────────────────────────────────────────────────────────

  private RobotState() {
    kinematics = new SwerveDriveKinematics(Drive.getModuleTranslations());
    for (int i = 0; i < 3; i++) {
      qStdDevs.set(i, 0, Math.pow(ODOMETRY_STD_DEVS.get(i, 0), 2));
    }
    AutoLogOutputManager.addObject(this);
  }

  // ── Odometry ──────────────────────────────────────────────────────────────

  /**
   * Called from {@code Drive.periodic()} for every odometry sample from the high-frequency thread.
   *
   * @param timestamp FPGA timestamp of the sample (seconds).
   * @param yaw Current raw gyro angle, or {@code null} if the gyro is disconnected (kinematics
   *     fallback is used instead).
   * @param wheelPositions Current module positions.
   */
  public void addOdometryObservation(
      double timestamp, Rotation2d yaw, SwerveModulePosition[] wheelPositions) {

    // Compute wheel twist since last sample
    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, wheelPositions);
    lastWheelPositions = wheelPositions;

    Pose2d lastOdometryPose = odometryPose;
    odometryPose = odometryPose.exp(twist);

    // Override heading from gyro when available, applying persistent offset
    if (yaw != null) {
      Rotation2d angle = yaw.plus(gyroOffset);
      odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
    }

    // Store pose in the interpolation buffer for later vision corrections
    poseBuffer.addSample(timestamp, odometryPose);

    // Propagate the odometry delta onto the vision-fused estimate
    Twist2d finalTwist = lastOdometryPose.log(odometryPose);
    estimatedPose = estimatedPose.exp(finalTwist);
  }

  // ── Vision ────────────────────────────────────────────────────────────────

  /**
   * Adds a new timestamped vision measurement, applying Kalman-weighted fusion with latency
   * compensation via the pose buffer.
   *
   * @param visionPose The vision-estimated robot pose.
   * @param timestamp FPGA timestamp when the frame was captured (seconds).
   * @param stdDevs Standard deviations for (x, y, θ).
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {

    // Discard measurements older than the buffer
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - POSE_BUFFER_SIZE_SEC > timestamp) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }

    var sample = poseBuffer.getSample(timestamp);
    if (sample.isEmpty()) return;

    // Compute transforms between odometry at sample time and now
    var sampleToOdometry = new Transform2d(sample.get(), odometryPose);
    var odometryToSample = new Transform2d(odometryPose, sample.get());

    // Shift estimated pose back to sample time
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSample);

    // Build diagonal Kalman gain — closed-form for continuous KF with A=0, C=I
    var r = new double[3];
    for (int i = 0; i < 3; i++) {
      r[i] = stdDevs.get(i, 0) * stdDevs.get(i, 0);
    }
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; row++) {
      double q = qStdDevs.get(row, 0);
      visionK.set(row, row, q == 0.0 ? 0.0 : q / (q + Math.sqrt(q * r[row])));
    }

    // Scale the transform from estimate→vision by the Kalman gain
    Transform2d transform = new Transform2d(estimateAtTime, visionPose);
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Reconstruct the fused estimate and shift forward to now
    estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometry);
  }

  // ── Pose reset ────────────────────────────────────────────────────────────

  /**
   * Resets both the odometry pose and estimated pose to {@code pose}. Updates the gyro offset so
   * that subsequent gyro readings map to the new frame.
   */
  public void resetPose(Pose2d pose) {
    gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
    estimatedPose = pose;
    odometryPose = pose;
    poseBuffer.clear();
  }

  // ── Velocity ──────────────────────────────────────────────────────────────

  /**
   * Called from {@code Drive.periodic()} each loop with the latest robot-relative measured speeds.
   */
  public void setRobotVelocity(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  /** Returns the latest robot-relative measured {@link ChassisSpeeds}. */
  public ChassisSpeeds getRobotVelocity() {
    return robotVelocity;
  }

  /** Returns the latest field-relative measured {@link ChassisSpeeds}. */
  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  // ── Accessors ─────────────────────────────────────────────────────────────

  /** Returns the current vision-fused estimated pose. */
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  /** Returns the rotation component of the current estimated pose. */
  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  /** Returns the raw odometry pose (no vision corrections). */
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  /**
   * Returns the estimated pose at a past timestamp using the pose buffer. Returns empty if the
   * timestamp is outside the buffer window.
   */
  public java.util.Optional<Pose2d> getEstimatedPoseAtTimestamp(double timestamp) {
    var oldOdometryPose = poseBuffer.getSample(timestamp);
    if (oldOdometryPose.isEmpty()) return java.util.Optional.empty();
    return java.util.Optional.of(
        estimatedPose.transformBy(new Transform2d(odometryPose, oldOdometryPose.get())));
  }
}
