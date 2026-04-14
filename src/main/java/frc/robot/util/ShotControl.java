// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.util;

/**
 * Shot-control utilities: polynomial distance-to-angle/RPM mapping and an operator-trimmable
 * hood-angle offset.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * private final ShotControl sc = new ShotControl();
 * // In periodic:
 * double angle = sc.getHoodAngle(distance);   // polynomial + trim offset
 * double rpm   = sc.getFlywheelRPM(distance); // polynomial
 * // Operator trim (bind to POV buttons):
 * sc.incrementHoodAngleOffset(+0.2);  // POV right
 * sc.incrementHoodAngleOffset(-0.2);  // POV left
 * }</pre>
 */
public class ShotControl {

  // ── Hood angle trim ───────────────────────────────────────────────────────────

  /** Cumulative operator trim added on top of the polynomial lookup (degrees). */
  private double hoodAngleOffsetDeg = -15;

  /**
   * Increments the hood-angle trim offset. Bind to POV right (+0.2°) and POV left (-0.2°) with
   * repeat-on-hold for in-match tuning without a redeploy.
   *
   * @param deltaDeg degrees to add (positive = up, negative = down)
   */
  public void incrementHoodAngleOffset(double deltaDeg) {
    hoodAngleOffsetDeg += deltaDeg;
  }

  /** Returns the current hood angle trim offset in degrees. */
  public double getHoodAngleOffset() {
    return hoodAngleOffsetDeg;
  }

  /** Resets the hood angle trim offset to zero. */
  public void resetHoodAngleOffset() {
    hoodAngleOffsetDeg = 0.0;
  }

  // ── Polynomial lookups ────────────────────────────────────────────────────────

  /**
   * Returns the commanded hood angle (degrees) for the given distance, including the operator trim
   * offset. The -15° mechanical offset is already baked into the hood's zero position.
   *
   * @param d distance to target in meters
   * @return hood angle in degrees
   */
  public double getHoodAngle(double d) {
    double angle =
        0.0018212122033993486 * d * d * d * d
            + 0.06500933459537382 * d * d * d
            + 0.8376141604081012 * d * d
            + 5.998258704910444 * d
            + 78.62901114203709;
    return 90 - angle + hoodAngleOffsetDeg;
  }

  /**
   * Returns the commanded flywheel RPM for the given distance.
   *
   * @param d distance to target in meters
   * @return flywheel RPM
   */
  public double getFlywheelRPM(double d) {
    return 0.020291685455647 * d * d * d * d
        + 0.4336905599652903 * d * d * d
        + -0.7957093913140425 * d * d
        + -134.8260015986782 * d
        + 577.0490626386577;
  }
}
