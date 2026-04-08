package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.rack.Rack;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.State;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Named commands for PathPlanner auto routines.
 *
 * <p>All auto commands automatically manage superstructure state transitions to prevent the
 * robot from hanging mid-auto (unlike the old system where AutoAim never ended).
 */
public class AutoCommands {

  // PID gains for angular (rotation) control during auto aim
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.0;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;

  /** Default time to spin up, aim, and feed during auto shooting (seconds). */
  private static final double SHOOT_TIMEOUT_SECONDS = 2.0;

  private AutoCommands() {}

  /**
   * Intake sequence: set INTAKING state, wait for rack to reach max extension, then return to
   * ACTIVE. This command always completes (no hanging), and returns to ACTIVE even if cancelled
   * mid-intake (via finallyDo).
   *
   * <p>Used in PathPlanner autos as: {@code NamedCommands.registerCommand("Intake Mode",
   * intakeMode(...))}
   */
  public static Command intakeMode(Superstructure superstructure, Rack rack) {
    return Commands.runOnce(() -> superstructure.setState(State.INTAKING), superstructure)
        .andThen(Commands.waitUntil(rack::isAtSetpoint)) // Wait for rack to reach MAX_POSITION
        .finallyDo(() -> superstructure.setState(State.ACTIVE)); // Always return to ACTIVE
  }

  /**
   * Complete auto shooting sequence: sets SHOOTING state, aims at target with a timeout, then
   * returns to ACTIVE. This ensures the command always terminates within {@code timeoutSeconds}
   * (no hanging like the old AutoAim).
   *
   * <p>Sequence:
   * <ol>
   *   <li>Set SHOOTING (spins up shooter and hood to match target distance)
   *   <li>AimAtPoint with timeout (PID-rotates chassis to face target)
   *   <li>Set ACTIVE (stops shooting, maintains deployed rack)
   * </ol>
   *
   * <p>Used in PathPlanner autos as: {@code NamedCommands.registerCommand("Shoot",
   * shootSequence(...))}
   */
  public static Command shootSequence(
      Superstructure superstructure,
      Drive drive,
      Supplier<Translation2d> targetSupplier,
      double timeoutSeconds) {
    return Commands.sequence(
        Commands.runOnce(() -> superstructure.setState(State.SHOOTING), superstructure),
        aimAtPoint(drive, targetSupplier).withTimeout(timeoutSeconds), // Guaranteed to end
        Commands.runOnce(() -> superstructure.setState(State.ACTIVE), superstructure));
  }

  /**
   * Convenience overload using the default timeout ({@link #SHOOT_TIMEOUT_SECONDS}).
   * Recommended for most autos.
   */
  public static Command shootSequence(
      Superstructure superstructure, Drive drive, Supplier<Translation2d> targetSupplier) {
    return shootSequence(superstructure, drive, targetSupplier, SHOOT_TIMEOUT_SECONDS);
  }

  /**
   * Pure drive command that aims the chassis at a field point without any superstructure state
   * dependency. Uses a ProfiledPIDController for smooth angular rotation. The command will
   * X-lock wheels when the robot is within tolerance of the target angle, making it energy
   * efficient while holding position.
   *
   * <p>This command never checks superstructure state, so it can be used in isolation or combined
   * with other commands via {@link edu.wpi.first.wpilibj2.command.Commands#sequence}.
   *
   * <p>Tolerance: 0.5 radians (~28.6 degrees).
   */
  public static Command aimAtPoint(Drive drive, Supplier<Translation2d> targetSupplier) {

    // ProfiledPID: smooth motion with trapezoidal velocity profile
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0, // No integral term
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI); // Handle wraparound at ±π
    angleController.setTolerance(0.5); // Within 0.5 rad is "at target"

    return Commands.run(
            () -> {
              // Calculate target angle from robot position to field target
              Translation2d robotToTarget =
                  targetSupplier.get().minus(drive.getPose().getTranslation());
              Rotation2d targetAngle = robotToTarget.getAngle();

              // Calculate angular velocity command via PID
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), targetAngle.getRadians());

              // Log telemetry for debugging/tuning
              Logger.recordOutput("Auto Align/robotRot", drive.getRotation().getDegrees());
              Logger.recordOutput("Auto Align/robotToTarget", robotToTarget);
              Logger.recordOutput("Auto Align/targetAngle", robotToTarget.getAngle());
              Logger.recordOutput("Auto Align/omega", omega);
              Logger.recordOutput("Auto Align/Chassis speeds", new ChassisSpeeds(0.0, 0.0, omega));
              Logger.recordOutput("Auto Align/Pos error", angleController.getPositionError());
              Logger.recordOutput("Auto Align/Vel error", angleController.getVelocityError());

              // Spin-in-place rotation; if near target, X-lock wheels instead
              if (Math.abs(omega) > 0.05) {
                drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omega)); // Rotate
              } else {
                drive.stopWithX(); // On target → X-lock to resist pushing
              }
            },
            drive)
        // Reset PID controller when command starts (avoids integrator buildup from previous runs)
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
        // Always X-lock wheels when done (safe brake position)
        .finallyDo(() -> drive.stopWithX());
  }
}
