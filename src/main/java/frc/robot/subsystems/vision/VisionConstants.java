// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "pv_LEFT"; // left
  public static String camera1Name = "pv_RIGHT"; // right

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.31435, -0.24779, 0.27251, new Rotation3d(0.0, Math.PI / 12, Math.PI / 2));
  public static Transform3d robotToCamera1 =
      new Transform3d(0.31435, 0.24779, 0.27251, new Rotation3d(0.0, Math.PI / 12, -Math.PI / 2));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // -------------------------------------------------------------------------
  // Northstar interface configuration
  // -------------------------------------------------------------------------

  /** Margin outside the field boundary within which robot poses are still accepted. */
  public static final double fieldBorderMargin = 0.5;

  /** Minimum and maximum acceptable Z component of an estimated robot pose. */
  public static final double zMin = -0.5;

  public static final double zMax = 1.0;

  /**
   * Configuration for a single Northstar camera coprocessor.
   *
   * @param cameraId USB serial ID of the camera (e.g. {@code "12345678"}).
   * @param resolutionWidth Horizontal pixel count of the capture resolution.
   * @param resolutionHeight Vertical pixel count of the capture resolution.
   * @param autoExposure {@code 1} to enable auto-exposure, {@code 0} to use fixed exposure.
   * @param exposure Fixed exposure value (used when {@code autoExposure == 0}).
   * @param gain Analogue gain applied by the camera driver.
   * @param denoise Denoise strength (0.0 = off, higher = more smoothing).
   * @param stdDevFactor Per-camera trust multiplier applied to both XY and theta std devs.
   * @param robotToCamera Rigid transform from the robot origin to this camera's optical centre.
   */
  public record NorthstarCameraConfig(
      String cameraId,
      int resolutionWidth,
      int resolutionHeight,
      int autoExposure,
      int exposure,
      double gain,
      double denoise,
      double stdDevFactor,
      Transform3d robotToCamera) {}

  /**
   * Northstar cameras installed on Obelix, indexed so that {@code northstarCameras[i]} matches the
   * Northstar instance running under the NT table {@code northstar_i}.
   */
  public static final NorthstarCameraConfig[] northstarCameras =
      new NorthstarCameraConfig[] {
        // Camera 0 – iPhone via Continuity Camera (testing)
        new NorthstarCameraConfig(
            "4", // iPhone camera index
            1280,
            720,
            3, // auto exposure ON (CAP_PROP_AUTO_EXPOSURE=3 on macOS)
            300,
            0.0,
            0.0,
            1.0,
            new Transform3d(0.0, 0.0, 0.0, new Rotation3d())),
        new NorthstarCameraConfig(
            "5", // iPhone camera index
            1280,
            720,
            3, // auto exposure ON (CAP_PROP_AUTO_EXPOSURE=3 on macOS)
            300,
            0.0,
            0.0,
            1.0,
            new Transform3d(0.0, 0.0, 0.0, new Rotation3d())),
      };

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0 // Camera 0
      };

  // Multipliers to apply for MegaTag 1 observations
  public static double linearStdDevMegatag1Factor =
      2.0; // Less trusted for XY due to ambiguity risk
  public static double angularStdDevMegatag1Factor = 2.0; // Rotation is valid (independent solve)

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // Per-meter/s of robot speed, inflate linearStdDev by this fraction.
  // At 3 m/s, a factor of 0.5 triples the std dev (1 + 0.5*3 = 2.5×) so
  // fast-moving
  // observations are trusted much less than stationary ones.
  public static double velocityLinearStdDevScaleFactor = 0.5;
}
