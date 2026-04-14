package frc.robot.util;

/**
 * Shot-control utilities: polynomial distance-to-angle/RPM mapping, named launch presets with
 * dashboard-tunable parameters, and an operator-trimmable hood-angle offset.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * private final ShotControl sc = new ShotControl();
 * // In periodic:
 * double angle = sc.getHoodAngle(distance);   // polynomial + offset
 * double rpm   = sc.getFlywheelRPM(distance); // polynomial
 * // Operator trim (bind to POV buttons):
 * sc.incrementHoodAngleOffset(+0.2);  // POV right
 * sc.incrementHoodAngleOffset(-0.2);  // POV left
 * }</pre>
 */
public class ShotControl {

  // ── Named presets ─────────────────────────────────────────────────────────────

  /**
   * A fixed-distance launch preset whose parameters are tunable from the dashboard at runtime (only
   * active when {@code Constants.tuningMode = true}).
   */
  public static class LaunchPreset {
    private final LoggedTunableNumber hoodAngleDeg;
    private final LoggedTunableNumber flywheelRPM;

    /**
     * @param name dashboard key suffix (e.g. {@code "Tower"})
     * @param defaultHoodAngle default hood angle in degrees
     * @param defaultRPM default flywheel RPM
     */
    public LaunchPreset(String name, double defaultHoodAngle, double defaultRPM) {
      hoodAngleDeg = new LoggedTunableNumber("ShotControl/" + name + "/HoodAngleDeg");
      flywheelRPM = new LoggedTunableNumber("ShotControl/" + name + "/FlywheelRPM");
      hoodAngleDeg.initDefault(defaultHoodAngle);
      flywheelRPM.initDefault(defaultRPM);
    }

    /** Hood angle in degrees for this preset. */
    public double getHoodAngleDeg() {
      return hoodAngleDeg.get();
    }

    /** Flywheel RPM for this preset. */
    public double getFlywheelRPM() {
      return flywheelRPM.get();
    }
  }

  /** Short-range / tower shot preset. */
  public static final LaunchPreset towerPreset = new LaunchPreset("Tower", 10.0, 2400.0);

  /** Trench / mid-range shot preset. */
  public static final LaunchPreset trenchPreset = new LaunchPreset("Trench", 18.0, 3200.0);

  // ── Hood angle trim ───────────────────────────────────────────────────────────

  /** Cumulative operator trim added on top of every polynomial / preset lookup (degrees). */
  private double hoodAngleOffsetDeg = 0.0;

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
   * offset.
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
    return 90 - angle - 15 + hoodAngleOffsetDeg;
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
