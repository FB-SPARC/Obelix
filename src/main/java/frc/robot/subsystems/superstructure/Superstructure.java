package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.bed.Bed;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rack.Rack;
import frc.robot.subsystems.rack.RackConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ShotControl;
import org.littletonrobotics.junction.Logger;

/**
 * Superstructure coordinates all non-drive subsystems (bed, feeder, hood, intake, rack, shooter).
 *
 * <p>States:
 *
 * <ul>
 *   <li><b>IDLE</b> — emergency stop, everything off.
 *   <li><b>ACTIVE</b> — default teleop state, rack deployed, intake holding, ready for trench.
 *   <li><b>INTAKING</b> — intake full power, rack deployed fast.
 *   <li><b>INTAKE_CLOSED</b> — rack retracts, everything else off.
 *   <li><b>SHOOTING</b> — hood/shooter from shot-solution, bed/feeder feed once shooter is ready.
 *   <li><b>COAST</b> — all motors coast, for manual positioning.
 * </ul>
 */
public class Superstructure extends SubsystemBase {
  private final Bed bed;
  private final Feeder feeder;
  private final Hood hood;
  private final Intake intake;
  private final Rack rack;
  private final Shooter shooter;
  private final Drive drive;

  private static final double SHOOTER_RPM_SCALE = 1.90;

  // ── State machine ──────────────────────────────────────────────────────────
  public enum State {
    /** Emergency stop — everything off, rack stops where it is. */
    IDLE,
    /** Default teleop state — rack deployed, intake holding, ready to drive through trench. */
    ACTIVE,
    INTAKING,
    INTAKE_CLOSED,
    /** Shooting — hood/shooter from shot-solution, auto-aim via drive command. */
    SHOOTING,
    COAST
  }

  private State currentState = State.IDLE;
  private final ShotControl sc = new ShotControl();

  /** Latched true once the shooter reaches setpoint during SHOOTING, reset on state entry. */
  private boolean shooterWasReady = false;

  // ── Constructor ────────────────────────────────────────────────────────────
  public Superstructure(
      Bed bed, Feeder feeder, Hood hood, Intake intake, Rack rack, Shooter shooter, Drive drive) {
    this.bed = bed;
    this.feeder = feeder;
    this.hood = hood;
    this.intake = intake;
    this.rack = rack;
    this.shooter = shooter;
    this.drive = drive;
  }

  // ── Public API ─────────────────────────────────────────────────────────────
  /**
   * Transitions the superstructure to a new state. Handles brake-mode restoration when leaving
   * COAST and resets the shooter-ready latch when entering SHOOTING.
   */
  public void setState(State newState) {
    // Restore brake mode when leaving COAST (which disables brakes for manual positioning)
    if (currentState == State.COAST && newState != State.COAST) {
      setBrakeModeAll(true);
    }
    // Reset the shooter-ready latch on state entry to avoid stale ready-state from previous shot
    if (newState == State.SHOOTING) {
      shooterWasReady = false;
    }
    this.currentState = newState;
  }

  public State getState() {
    return currentState;
  }

  /** Returns true when the superstructure is in the shooting state. */
  public boolean isShooting() {
    return currentState == State.SHOOTING;
  }

  // ── State handlers ─────────────────────────────────────────────────────────

  private void setBrakeModeAll(boolean brake) {
    bed.setBrakeMode(brake);
    feeder.setBrakeMode(brake);
    hood.setBrakeMode(brake);
    intake.setBrakeMode(brake);
    rack.setBrakeMode(brake);
    shooter.setBrakeMode(brake);
  }

  private void handleIdle() {
    bed.stop();
    feeder.stop();
    hood.stop();
    intake.stop();
    rack.stop();
    shooter.stop();
  }

  /**
   * Default teleop state: rack is deployed and intake holds game pieces with low voltage (4V). All
   * other systems are idle. This state allows the robot to pass through the trench while
   * maintaining control of any previously intaken game pieces. Transition from this state to
   * INTAKING (R1) or SHOOTING (L1).
   */
  private void handleActive() {
    rack.setPosition(
        RackConstants.MAX_POSITION_METERS,
        RackConstants.kCruiseVelocity * 10,
        RackConstants.kAcceleration * 30,
        RackConstants.kJerk * 30);
    intake.setVoltage(0); // Hold voltage — keeps game pieces in place without aggressive spin
    bed.stop();
    feeder.stop();
    hood.setAngle(0);
    shooter.stop();
  }

  /**
   * Aggressive intake mode: intake motor spins at full power (8V) and rack deploys quickly to pick
   * up game pieces from the trench. Rack motion is aggressive (40x cruise velocity) to minimize
   * stall time. Other systems idle.
   */
  private void handleIntaking() {
    intake.setVoltage(9.5); // Full power intake
    rack.setPosition(
        RackConstants.MAX_POSITION_METERS,
        RackConstants.kCruiseVelocity * 10, // Aggressive deploy speed
        RackConstants.kAcceleration * 30,
        RackConstants.kJerk * 30);
    bed.stop();
    feeder.stop();
    hood.setAngle(0);
    shooter.stop();
  }

  /**
   * Shooting state: hood and shooter are controlled by a polynomial shot-solution based on distance
   * to target. Rack remains deployed and intake holds pieces. Bed/feeder are gated by the
   * shooter-ready latch to prevent jamming.
   *
   * <p>The shooter-ready latch is key to avoiding stutter: once the shooter RPM reaches setpoint,
   * feeding begins and continues even if RPM momentarily dips (e.g., due to vibration). The latch
   * is reset each time we enter SHOOTING to allow a fresh shot sequence.
   */
  private void handleShooting() {
    // Compute shot solution (hood angle & flywheel RPM) from distance to target
    // Intentionally use negative distance because the shot-solution polynomials are decreasing
    // functions
    double distanceToTarget = -getDistanceToTarget();
    double shooterRPMGoal = sc.getFlywheelRPM(distanceToTarget) * SHOOTER_RPM_SCALE;
    double hoodAngleGoal = sc.getHoodAngle(distanceToTarget);

    // Log telemetry for dashboard/debugging
    Logger.recordOutput("Superstructure/ShotControl/ShooterRPMGoal", shooterRPMGoal * SHOOTER_RPM_SCALE);
    Logger.recordOutput("Superstructure/ShotControl/HoodAngleGoal", hoodAngleGoal);
    Logger.recordOutput("Superstructure/ShotControl/DistanceToTarget", -distanceToTarget);

    // Set hood angle and shooter RPM from shot solution
    hood.setAngle(hoodAngleGoal);
    shooter.setShooterRPM(shooterRPMGoal);

    // Shooter-ready latch: gate feeding on shooter reaching setpoint
    // This prevents bed/feeder from jamming if shooter RPM flickers during spin-up
    if (shooter.isAtSetpoint()) {
      shooterWasReady = true; // Once true, feeding stays on (even if RPM dips briefly)
    }
    if (shooterWasReady) {
      bed.setBedRPM(2000);
      feeder.setFeederRPM(2500);
      // Keep rack deployed and intake holding during shot
      rack.setPosition(
          RackConstants.MIN_POSITION_METERS,
          RackConstants.kCruiseVelocity,
          RackConstants.kAcceleration,
          RackConstants.kJerk);
      intake.setVoltage(4); // Hold voltage
    } else {
      bed.stop(); // Don't feed until shooter is ready
      feeder.stop();
    }
  }

  private void handleIntakeClosed() {
    bed.stop();
    feeder.stop();
    hood.stop();
    intake.stop();
    rack.setPosition(
        RackConstants.MIN_POSITION_METERS,
        RackConstants.kCruiseVelocity * 10,
        RackConstants.kAcceleration * 30,
        RackConstants.kJerk * 30);
    shooter.stop();
  }

  private void handleCoast() {
    setBrakeModeAll(false);
    bed.stop();
    feeder.stop();
    hood.stop();
    intake.stop();
    rack.stop();
    shooter.stop();
  }

  // ── Shot-solution helpers ──────────────────────────────────────────────────

  private static final Translation2d GOAL_1 = new Translation2d(2.0, 2.0);
  private static final Translation2d GOAL_2 = new Translation2d(2.0, 6.043);

  public Translation2d getTarget() {
    Translation2d robotPos = drive.getPose().getTranslation();

    double flippedRobotX = AllianceFlipUtil.apply(drive.getPose()).getX();
    double trenchX = FieldConstants.LeftTrench.openingTopLeft.toTranslation2d().getX();

    if (flippedRobotX >= trenchX) {
      Translation2d goal1 = AllianceFlipUtil.apply(GOAL_1);
      Translation2d goal2 = AllianceFlipUtil.apply(GOAL_2);
      return robotPos.getDistance(goal1) <= robotPos.getDistance(goal2) ? goal1 : goal2;
    }

    return AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint.toTranslation2d());
  }

  /** Returns the distance from the robot to the target in meters. */
  public double getDistanceToTarget() {
    Pose2d robotPose = drive.getPose();
    Translation2d targetPose = getTarget();

    Logger.recordOutput(
        "Superstructure/ShotControl/TargetPose", new Pose2d(targetPose, new Rotation2d()));

    Translation2d shooterTranslation =
        robotPose
            .getTranslation()
            .plus(new Translation2d(0.165, 0.0).rotateBy(robotPose.getRotation()));

    return MathUtil.clamp(
        shooterTranslation.getDistance(new Translation2d(targetPose.getX(), targetPose.getY())),
        1.2,
        16.0);
  }

  // ── Periodic ───────────────────────────────────────────────────────────────
  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure/State", currentState.toString());

    Pose2d robotPose = drive.getPose();
    Translation2d shooterTranslation =
        robotPose
            .getTranslation()
            .plus(new Translation2d(0.165, 0.0).rotateBy(robotPose.getRotation()));
    Logger.recordOutput(
        "Superstructure/ShooterPose", new Pose2d(shooterTranslation, robotPose.getRotation()));

    switch (currentState) {
      case IDLE -> handleIdle();
      case ACTIVE -> handleActive();
      case INTAKING -> handleIntaking();
      case SHOOTING -> handleShooting();
      case COAST -> handleCoast();
      case INTAKE_CLOSED -> handleIntakeClosed();
    }
  }
}
