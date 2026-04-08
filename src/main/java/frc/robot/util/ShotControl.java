package frc.robot.util;

public class ShotControl {
  public double getHoodAngle(double d) {
    double angle =
        0.35581726946790054 * d * d * d
            + 4.6344950564566725 * d * d
            + 21.06980499578108 * d
            + 94.16710557480171;
    return 90 - angle - 15;
  }

  public double getFlywheelRPM(double d) {
    return 3.6852704478413747 * d * d * d
        + 41.3262820011483 * d * d
        + 36.76601699587665 * d
        + 753.3349423571889;
  }
}
