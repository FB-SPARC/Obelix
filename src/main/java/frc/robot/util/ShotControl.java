package frc.robot.util;

public class ShotControl {
  public double getHoodAngle(double d) {
    double angle =
        0.0018212122033993486 * d * d * d * d
            + 0.06500933459537382 * d * d * d
            + 0.8376141604081012 * d * d
            + 5.998258704910444 * d
            + 78.62901114203709;
    return 90 - angle - 15;
  }

  public double getFlywheelRPM(double d) {
    return 0.020291685455647 * d * d * d * d
        + 0.4336905599652903 * d * d * d
        + -0.7957093913140425 * d * d
        + -134.8260015986782 * d
        + 577.0490626386577;
  }
}
