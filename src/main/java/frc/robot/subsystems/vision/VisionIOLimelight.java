// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/**
 * IO implementation for real Limelight hardware.
 *
 * <p>Blocking NetworkTables operations (orientation publish, IMU mode publish, {@code flush()}, and
 * {@code readQueue()} draining) are handled by {@link LimelightThread}, which runs on a dedicated
 * 100 Hz background thread. This class only reads the pre-buffered results from that thread on the
 * main robot loop.
 */
public class VisionIOLimelight implements VisionIO {
  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final LimelightThread limelightThread;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight (e.g. {@code "limelight-left"}).
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);

    limelightThread = new LimelightThread(name, rotationSupplier);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Connection check — no NT flush needed here; latency topic updates independently
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Target observation (simple scalar reads — fast, no queue)
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Drain observations buffered by the background thread (no NT operations here)
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = limelightThread.getObservationsAndClear(tagIds);

    // Copy pose observations to inputs array
    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

    // Copy tag IDs to inputs array
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
