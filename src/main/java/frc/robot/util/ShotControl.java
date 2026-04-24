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
        0.0008349830255554643 * d * d * d * d
            + 0.03092015428402132 * d * d * d
            + 0.42602630891609855 * d * d
            + 4.0766181857714585 * d
            + 77.77541767893516;
    return 90 - angle + hoodAngleOffsetDeg;
  }

  public double getTime(double d) {
    return (((-1.103729650689687e-05 * d - 0.0008641281622582973) * d - 0.02179754466779854) * d
                - 0.2964218858537879)
            * d
        + 0.5754113225246014;
  }

  /**
   * Returns the commanded flywheel RPM for the given distance.
   *
   * @param d distance to target in meters
   * @return flywheel RPM
   */
  public double getFlywheelRPM(double d) {
    return -0.002875852992594171 * d * d * d * d
        - 0.3583547252436268 * d * d * d
        - 10.483815724034672 * d * d
        - 183.33024110519074 * d
        + 543.3690801087693;
  }
}
