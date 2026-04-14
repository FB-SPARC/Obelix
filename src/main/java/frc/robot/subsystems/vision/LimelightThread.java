// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

/**
 * Runs blocking Limelight NetworkTables operations on a 100 Hz background thread, keeping them off
 * the main 50 Hz robot loop.
 *
 * <p>The background thread:
 *
 * <ul>
 *   <li>Publishes the current robot orientation for MegaTag 2
 *   <li>Sets the IMU mode (external seed when disabled, fused when enabled)
 *   <li>Calls {@code NetworkTableInstance.flush()} — the main source of loop overruns
 *   <li>Drains {@code megatag1} and {@code megatag2} read queues into a lock-protected list
 * </ul>
 *
 * <p>{@link #getObservationsAndClear()} is called from the main thread in {@code
 * VisionIOLimelight.updateInputs()} and does no NT operations — it only acquires the lock and
 * drains the buffered list.
 */
public class LimelightThread {
  private static final double THREAD_PERIOD_SECONDS = 0.01; // 100 Hz

  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;
  private final IntegerPublisher imuModePublisher;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  private final ReentrantLock observationsLock = new ReentrantLock();
  private final List<PoseObservation> bufferedObservations = new ArrayList<>();
  private final Set<Integer> bufferedTagIds = new HashSet<>();

  private final Notifier notifier;

  /**
   * Creates a new LimelightThread for one camera.
   *
   * @param tableName the NT table name configured on the Limelight (e.g. {@code "limelight-left"})
   * @param rotationSupplier supplier for the current estimated robot heading (for MegaTag 2)
   */
  public LimelightThread(String tableName, Supplier<Rotation2d> rotationSupplier) {
    this.rotationSupplier = rotationSupplier;

    var table = NetworkTableInstance.getDefault().getTable(tableName);
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    imuModePublisher = table.getIntegerTopic("imumode_set").publish();
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

    notifier = new Notifier(this::threadPeriodic);
    notifier.setName("LimelightThread-" + tableName);
    notifier.startPeriodic(THREAD_PERIOD_SECONDS);
  }

  /** Background thread body — runs at 100 Hz. All NT operations happen here. */
  private void threadPeriodic() {
    // Publish current robot orientation for MegaTag 2 heading seed
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});

    // Set IMU mode 1 = EXTERNAL_SEED — seeds internal IMU from robot orientation every frame.
    imuModePublisher.accept(1);

    // Flush NT — this is the blocking call we're moving off the main thread
    NetworkTableInstance.getDefault().flush();

    // Drain read queues and buffer the observations
    List<PoseObservation> newObs = new ArrayList<>();
    Set<Integer> newTagIds = new HashSet<>();
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        newTagIds.add((int) rawSample.value[i]);
      }
      newObs.add(
          new PoseObservation(
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
              parsePose(rawSample.value),
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,
              (int) rawSample.value[7],
              rawSample.value[9],
              PoseObservationType.MEGATAG_1));
    }
    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        newTagIds.add((int) rawSample.value[i]);
      }
      newObs.add(
          new PoseObservation(
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
              parsePose(rawSample.value),
              0.0,
              (int) rawSample.value[7],
              rawSample.value[9],
              PoseObservationType.MEGATAG_2));
    }

    if (!newObs.isEmpty() || !newTagIds.isEmpty()) {
      observationsLock.lock();
      try {
        bufferedObservations.addAll(newObs);
        bufferedTagIds.addAll(newTagIds);
      } finally {
        observationsLock.unlock();
      }
    }
  }

  /**
   * Called from the main thread in {@code VisionIOLimelight.updateInputs()}. Atomically drains and
   * returns all observations buffered since the last call, and populates {@code outTagIds} with the
   * seen tag IDs. Does <em>no</em> NT operations.
   */
  public List<PoseObservation> getObservationsAndClear(Set<Integer> outTagIds) {
    observationsLock.lock();
    try {
      List<PoseObservation> result = new ArrayList<>(bufferedObservations);
      outTagIds.addAll(bufferedTagIds);
      bufferedObservations.clear();
      bufferedTagIds.clear();
      return result;
    } finally {
      observationsLock.unlock();
    }
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
