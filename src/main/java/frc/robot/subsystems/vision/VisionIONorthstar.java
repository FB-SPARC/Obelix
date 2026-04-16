// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

/**
 * IO implementation for a Northstar coprocessor running on a Raspberry Pi (or similar).
 *
 * <p>Each Northstar instance publishes AprilTag observations over NT4 under the table {@code
 * northstar_<index>}. This class subscribes to the observation queue, decodes the raw frame format
 * used by Northstar's AprilTag pipeline, and exposes the results as {@link
 * VisionIO.PoseObservation} objects compatible with the rest of the vision pipeline.
 *
 * <h3>Frame format</h3>
 *
 * <p>Each double-array observation follows the schema produced by Northstar:
 *
 * <pre>
 * values[0]  — pose count (1 = multi-tag solve, 2 = single-tag with two candidates)
 *
 * For multi-tag (values[0] == 1):
 *   values[1]       — reprojection error
 *   values[2..4]    — camera position  (x, y, z) in metres
 *   values[5..8]    — camera rotation  (Quaternion w, x, y, z)
 *   values[9..]     — tag data, 10 values per tag:
 *                       [tagId, corner0x, corner0y, corner1x, corner1y,
 *                        corner2x, corner2y, corner3x, corner3y, 0]
 *
 * For single-tag (values[0] == 2):
 *   values[1]       — error for pose 0
 *   values[2..4]    — camera position  (pose 0)
 *   values[5..8]    — camera rotation  (pose 0)
 *   values[9]       — error for pose 1
 *   values[10..12]  — camera position  (pose 1)
 *   values[13..16]  — camera rotation  (pose 1)
 *   values[17..]    — tag data (same 10-value layout as above)
 * </pre>
 */
public class VisionIONorthstar implements VisionIO {

  private final String deviceId;
  private final Transform3d cameraToRobot;
  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<AprilTagLayoutType> aprilTagLayoutSupplier;
  private AprilTagLayoutType lastAprilTagLayout = null;

  private final DoubleArraySubscriber observationSubscriber;
  private final IntegerSubscriber fpsSubscriber;

  // NT publishers for Northstar configuration
  private final edu.wpi.first.networktables.StringPublisher eventNamePublisher;
  private final edu.wpi.first.networktables.IntegerPublisher matchTypePublisher;
  private final edu.wpi.first.networktables.IntegerPublisher matchNumberPublisher;
  private final edu.wpi.first.networktables.IntegerPublisher timestampPublisher;
  private final edu.wpi.first.networktables.StringPublisher tagLayoutPublisher;

  /**
   * Creates a new {@code VisionIONorthstar}.
   *
   * @param index Camera index (0-based). Must match the Northstar instance's NT table suffix, e.g.
   *     {@code 0} → {@code northstar_0}.
   * @param aprilTagLayoutSupplier Supplier for the current AprilTag layout type, published to
   *     Northstar whenever it changes so the pipeline uses the right tag map.
   * @param rotationSupplier Supplier for the robot's current estimated heading, used to
   *     disambiguate single-tag pose candidates.
   */
  public VisionIONorthstar(
      int index,
      Supplier<AprilTagLayoutType> aprilTagLayoutSupplier,
      Supplier<Rotation2d> rotationSupplier) {
    this.aprilTagLayoutSupplier = aprilTagLayoutSupplier;
    this.rotationSupplier = rotationSupplier;

    NorthstarCameraConfig camera = northstarCameras[index];
    this.deviceId = "northstar_" + index;
    // Invert robot-to-camera so we can go camera → robot during pose recovery
    this.cameraToRobot = camera.robotToCamera().inverse();

    var ntInstance = NetworkTableInstance.getDefault();
    var northstarTable = ntInstance.getTable(deviceId);
    var configTable = northstarTable.getSubTable("config");

    // ── Publish static camera configuration ──────────────────────────────────
    configTable.getStringTopic("camera_id").publish().set(camera.cameraId());
    configTable.getIntegerTopic("camera_resolution_width").publish().set(camera.resolutionWidth());
    configTable
        .getIntegerTopic("camera_resolution_height")
        .publish()
        .set(camera.resolutionHeight());
    configTable.getIntegerTopic("camera_auto_exposure").publish().set(camera.autoExposure());
    configTable.getIntegerTopic("camera_exposure").publish().set(camera.exposure());
    configTable.getDoubleTopic("camera_gain").publish().set(camera.gain());
    configTable.getDoubleTopic("camera_denoise").publish().set(camera.denoise());
    configTable.getDoubleTopic("fiducial_size_m").publish().set(FieldConstants.aprilTagWidth);

    // ── Dynamic / per-match config publishers ────────────────────────────────
    timestampPublisher = configTable.getIntegerTopic("timestamp").publish();
    tagLayoutPublisher = configTable.getStringTopic("tag_layout").publish();
    eventNamePublisher = configTable.getStringTopic("event_name").publish();
    matchTypePublisher = configTable.getIntegerTopic("match_type").publish();
    matchNumberPublisher = configTable.getIntegerTopic("match_number").publish();

    // Tag layout is published dynamically in updateInputs() so Northstar picks
    // up any mid-match layout changes (e.g. switching between OFFICIAL / NONE).

    // ── Subscribe to observation output ──────────────────────────────────────
    var outputTable = northstarTable.getSubTable("output");
    observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(5),
                PubSubOption.periodic(0.01));
    fpsSubscriber = outputTable.getIntegerTopic("fps_apriltags").subscribe(0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // ── Keep Northstar informed of match metadata & roboRIO clock ────────────
    timestampPublisher.set(WPIUtilJNI.getSystemTime() / 1_000_000L);
    eventNamePublisher.set(DriverStation.getEventName());
    matchTypePublisher.set(DriverStation.getMatchType().ordinal());
    matchNumberPublisher.set(DriverStation.getMatchNumber());

    // ── Publish tag layout only when it changes ───────────────────────────────
    AprilTagLayoutType currentLayout = aprilTagLayoutSupplier.get();
    if (currentLayout != lastAprilTagLayout) {
      lastAprilTagLayout = currentLayout;
      tagLayoutPublisher.set(currentLayout.getLayoutString());
    }

    // ── NT connection check ───────────────────────────────────────────────────
    inputs.connected = false;
    for (var client : NetworkTableInstance.getDefault().getConnections()) {
      if (client.remote_id.startsWith(deviceId)) {
        inputs.connected = true;
        break;
      }
    }

    // ── Parse observation queue ───────────────────────────────────────────────
    var queue = observationSubscriber.readQueue();
    List<PoseObservation> observations = new ArrayList<>(queue.length);
    Set<Integer> tagIds = new HashSet<>();

    for (var entry : queue) {
      double timestamp = entry.timestamp / 1_000_000.0; // µs → s
      double[] values = entry.value;

      Optional<Pose3d> cameraPoseOpt = parseCameraPose(values, tagIds);
      if (cameraPoseOpt.isEmpty()) continue;

      Pose3d cameraPose = cameraPoseOpt.get();
      Pose3d robotPose = cameraPose.transformBy(cameraToRobot);

      // Bounds-check: reject poses outside the field + margin
      if (robotPose.getX() < -fieldBorderMargin
          || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
          || robotPose.getY() < -fieldBorderMargin
          || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin
          || robotPose.getZ() < zMin
          || robotPose.getZ() > zMax) {
        continue;
      }

      // Collect visible tag poses for average-distance calculation
      int tagStartIdx = ((int) values[0] == 1) ? 9 : 17;
      List<Pose3d> tagPoses = new ArrayList<>();
      int tagCount = 0;
      for (int i = tagStartIdx; i < values.length; i += 10) {
        int tagId = (int) values[i];
        tagIds.add(tagId);
        aprilTagLayout.getTagPose(tagId).ifPresent(tagPoses::add);
        tagCount++;
      }
      if (tagPoses.isEmpty()) continue;

      double totalDist = 0.0;
      for (Pose3d tp : tagPoses) {
        totalDist += tp.getTranslation().getDistance(cameraPose.getTranslation());
      }
      double avgDist = totalDist / tagPoses.size();

      // Compute ambiguity ratio for single-tag; multi-tag is unambiguous (0.0)
      double ambiguity = computeAmbiguity(values);

      observations.add(
          new PoseObservation(
              timestamp,
              robotPose,
              ambiguity,
              tagCount,
              avgDist,
              // Northstar does a full 3D PnP solve, most similar to PhotonVision
              PoseObservationType.NORTHSTAR));
    }

    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
    // Northstar doesn't provide a simple tx/ty target; leave at zero
    inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
  }

  // ── Helpers ────────────────────────────────────────────────────────────────

  /**
   * Decodes a Northstar observation frame and returns the selected camera pose.
   *
   * <p>For single-tag frames the two candidate poses are disambiguated using the odometry heading
   * supplied via {@link #rotationSupplier}.
   *
   * @param values Raw double-array from the NT observation topic.
   * @param tagIds Mutable set to which detected tag IDs are added.
   * @return The selected camera pose, or {@link Optional#empty()} if the frame should be skipped.
   */
  private Optional<Pose3d> parseCameraPose(double[] values, Set<Integer> tagIds) {
    if (values.length == 0 || (int) values[0] == 0) return Optional.empty();

    switch ((int) values[0]) {
      case 1:
        {
          // Multi-tag solve — a single well-determined camera pose
          return Optional.of(buildPose(values, 2));
        }
      case 2:
        {
          // Single-tag — two candidate solutions; pick by rotation proximity
          double error0 = values[1];
          double error1 = values[9];

          // Only attempt disambiguation when one solution is clearly better
          if (error0 >= error1 * maxAmbiguity && error1 >= error0 * maxAmbiguity) {
            // Too ambiguous; skip this frame
            return Optional.empty();
          }

          Pose3d camPose0 = buildPose(values, 2);
          Pose3d camPose1 = buildPose(values, 10);
          Pose3d robotPose0 = camPose0.transformBy(cameraToRobot);
          Pose3d robotPose1 = camPose1.transformBy(cameraToRobot);

          Rotation2d currentRot = rotationSupplier.get();
          double diff0 =
              Math.abs(currentRot.minus(robotPose0.toPose2d().getRotation()).getRadians());
          double diff1 =
              Math.abs(currentRot.minus(robotPose1.toPose2d().getRotation()).getRadians());

          return Optional.of(diff0 <= diff1 ? camPose0 : camPose1);
        }
      default:
        return Optional.empty();
    }
  }

  /**
   * Constructs a {@link Pose3d} from a Northstar frame array starting at {@code offset}.
   *
   * <p>Layout: {@code [x, y, z, qw, qx, qy, qz]}.
   */
  private static Pose3d buildPose(double[] values, int offset) {
    return new Pose3d(
        values[offset],
        values[offset + 1],
        values[offset + 2],
        new Rotation3d(
            new Quaternion(
                values[offset + 3], values[offset + 4], values[offset + 5], values[offset + 6])));
  }

  /**
   * Returns the ambiguity ratio for the frame. Multi-tag frames have zero ambiguity. For single-tag
   * the ratio is {@code min(e0,e1) / max(e0,e1)}, clamped to [0, 1].
   */
  private static double computeAmbiguity(double[] values) {
    if ((int) values[0] != 2) return 0.0;
    double e0 = values[1];
    double e1 = values[9];
    if (e0 <= 0.0 || e1 <= 0.0) return 0.0;
    double lo = Math.min(e0, e1);
    double hi = Math.max(e0, e1);
    return lo / hi;
  }

  /**
   * Returns the most recently reported frames-per-second from the Northstar AprilTag pipeline.
   * Useful for dashboards / diagnostics.
   */
  public long getFps() {
    return fpsSubscriber.get();
  }
}
