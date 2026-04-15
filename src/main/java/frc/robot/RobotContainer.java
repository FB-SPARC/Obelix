// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveTrajectory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.bed.Bed;
import frc.robot.subsystems.bed.BedIO;
import frc.robot.subsystems.bed.BedIOSim;
import frc.robot.subsystems.bed.BedIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.rack.Rack;
import frc.robot.subsystems.rack.RackIO;
import frc.robot.subsystems.rack.RackIOSim;
import frc.robot.subsystems.rack.RackIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.State;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@ExtensionMethod({frc.robot.util.TriggerUtil.class})
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Bed bed;
  private final Feeder feeder;
  private final Hood hood;
  private final Intake intake;
  private final Rack rack;
  private final Shooter shooter;
  private final Superstructure superstructure;

  // Vision is intentionally held alive here; it self-registers pose callbacks via its periodic().
  @SuppressWarnings("unused")
  private final Vision vision;

  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);
  private final Alert controllerDisconnectedAlert =
      new Alert("Driver controller disconnected (port 0).", AlertType.kError);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final Map<Command, List<Trajectory<SwerveSample>>> autoTrajectories = new HashMap<>();
  private Command lastDisplayedAuto = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        bed = new Bed(new BedIOTalonFX());
        feeder = new Feeder(new FeederIOTalonFX());
        hood = new Hood(new HoodIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        rack = new Rack(new RackIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());
        vision =
            new Vision(
                new VisionIOLimelight("limelight-left", RobotState.getInstance()::getRotation),
                new VisionIOLimelight("limelight-right", RobotState.getInstance()::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        bed = new Bed(new BedIOSim());
        feeder = new Feeder(new FeederIOSim());
        hood = new Hood(new HoodIOSim());
        intake = new Intake(new IntakeIOSim());
        rack = new Rack(new RackIOSim());
        shooter = new Shooter(new ShooterIOSim());

        vision = new Vision(new VisionIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        bed = new Bed(new BedIO() {});
        feeder = new Feeder(new FeederIO() {});
        hood = new Hood(new HoodIO() {});
        intake = new Intake(new IntakeIO() {});
        rack = new Rack(new RackIO() {});
        shooter = new Shooter(new ShooterIO() {});
        vision = new Vision(new VisionIO() {});
        break;
    }

    // Create superstructure (coordinates all non-drive subsystems + drive)
    superstructure = new Superstructure(bed, feeder, hood, intake, rack, shooter, drive);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("None", Commands.none());
    autoChooser.addOption("Left Double Swing", leftDoubleSwingAuto());
    autoChooser.addOption("Right Double Swing", rightDoubleSwingAuto());
    autoChooser.addOption("Mid", midAuto());
    autoChooser.addOption("Mid Depot", midDepotAuto());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Rumble controller for 0.5s at teleop start
    RobotModeTriggers.teleop()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 1.0)),
                new edu.wpi.first.wpilibj2.command.WaitCommand(0.5),
                Commands.runOnce(
                    () -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0))));

    // ── Superstructure state bindings ──────────────────────────────────────
    // Triangle: reset rack encoder for calibration
    // controller.triangle().onTrue(Commands.runOnce(() -> rack.resetEncoder(), rack));

    // R1: Intake control (press → INTAKING, release → ACTIVE if was intaking)
    controller
        .R1()
        .onTrue(Commands.runOnce(() -> superstructure.setState(State.INTAKING), superstructure));
    controller
        .R1()
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (superstructure.getState() == State.INTAKING)
                    superstructure.setState(State.ACTIVE);
                },
                superstructure));

    controller
        .options()
        .onTrue(
            Commands.runOnce(() -> superstructure.setState(State.INTAKE_CLOSED), superstructure));

    // ── Shooting mode ─────────────────────────────────────────────────────
    final double DEADBAND = 0.1;

    // L1: Shooting control (press → SHOOTING, release → ACTIVE if was shooting)
    // L1 is the primary shot authority — holding it enables both state and drive auto-aim
    controller
        .L1()
        .onTrue(Commands.runOnce(() -> superstructure.setState(State.SHOOTING), superstructure));
    controller
        .L1()
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (superstructure.isShooting()) superstructure.setState(State.ACTIVE);
                },
                superstructure));

    // Drive auto-aim while L1 held: PID angular control to target.
    // Left stick Y/X supply translational velocity (squared input for precision).
    // Stick released → stationary shot (X-lock wheels on-target).
    // Stick input → moving shot (shoot-on-the-move with PID rotation).
    controller
        .L1()
        .whileTrue(
            DriveCommands.joystickDriveAimAtPoint(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                superstructure::getTarget,
                () ->
                    MathUtil.applyDeadband(
                            Math.hypot(controller.getLeftX(), controller.getLeftY()), DEADBAND)
                        > 0.0));

    // Right stick: Cancel shot (e.g., adjust after release, recoil compensation, or abort)
    // Any right stick input while shooting → ACTIVE (maintains deployed state, stops shooting)
    new Trigger(
            () ->
                MathUtil.applyDeadband(Math.abs(controller.getRightX()), DEADBAND) > 0.0
                    || MathUtil.applyDeadband(Math.abs(controller.getRightY()), DEADBAND) > 0.0)
        .and(new Trigger(superstructure::isShooting))
        .onTrue(Commands.runOnce(() -> superstructure.setState(State.ACTIVE), superstructure));

    // On teleop enable: set superstructure to ACTIVE (ready-to-drive state)
    RobotModeTriggers.teleop()
        .onTrue(Commands.runOnce(() -> superstructure.setState(State.ACTIVE), superstructure));
    // Touchpad: Emergency stop (true panic button)
    // Shuts down all motors immediately regardless of state
    controller
        .touchpad()
        .onTrue(Commands.runOnce(() -> superstructure.setState(State.IDLE), superstructure));
  }

  //   public void teleopInit() {
  //     if (superstructure != null) superstructure.setState(State.ACTIVE);
  //   }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Called periodically from {@link Robot#robotPeriodic()}. */
  public void periodic() {
    // Update controller disconnection alert
    controllerDisconnectedAlert.set(
        !DriverStation.isJoystickConnected(controller.getHID().getPort()));

    // Display the selected auto's trajectory on the Field2d widget
    Command selectedAuto = autoChooser.get();
    if (selectedAuto != lastDisplayedAuto) {
      lastDisplayedAuto = selectedAuto;
      List<Trajectory<SwerveSample>> trajs = autoTrajectories.getOrDefault(selectedAuto, List.of());
      List<Pose2d> allPoses = new ArrayList<>();
      boolean mirror = shouldMirror();
      for (var traj : trajs) {
        for (var sample : traj.samples()) {
          allPoses.add(mirror ? sample.flipped().getPose() : sample.getPose());
        }
      }
      Pose2d[] posesArray = allPoses.toArray(Pose2d[]::new);
      drive.setAutoTrajectory(posesArray);
      Logger.recordOutput("Odometry/AutoTrajectory", posesArray);
    }
  }

  // ── Choreo auto helpers ───────────────────────────────────────────────────

  private boolean shouldMirror() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  /**
   * Loads a Choreo trajectory by name and returns a command to follow it. If {@code resetPose} is
   * true, resets odometry to the trajectory's initial pose before driving. The loaded trajectory is
   * also appended to {@code accumulator} for dashboard visualization.
   */
  private Command followTrajectory(
      String name, boolean resetPose, List<Trajectory<SwerveSample>> accumulator) {
    var rawOpt = Choreo.loadTrajectory(name);
    if (rawOpt.isEmpty()) {
      return Commands.print("WARNING: Choreo trajectory '" + name + "' not found.");
    }
    @SuppressWarnings("unchecked")
    Trajectory<SwerveSample> trajectory = (Trajectory<SwerveSample>) rawOpt.get();
    accumulator.add(trajectory);
    Command resetCmd =
        resetPose
            ? Commands.runOnce(
                () ->
                    trajectory
                        .getInitialSample(shouldMirror())
                        .ifPresent(s -> drive.setPose(s.getPose())))
            : Commands.none();
    return resetCmd.andThen(new DriveTrajectory(trajectory, drive, this::shouldMirror));
  }

  /** Registers an auto command with its trajectories for dashboard visualization. */
  private Command registerAutoTrajectories(
      Command auto, List<Trajectory<SwerveSample>> trajectories) {
    autoTrajectories.put(auto, trajectories);
    return auto;
  }

  // ── Auto routines ─────────────────────────────────────────────────────────

  private Command leftDoubleSwingAuto() {
    List<Trajectory<SwerveSample>> trajs = new ArrayList<>();
    return registerAutoTrajectories(
        Commands.sequence(
            AutoCommands.intakeMode(superstructure, rack),
            followTrajectory("udl1", true, trajs),
            followTrajectory("udl2", false, trajs),
            AutoCommands.shootSequence(superstructure, drive, superstructure::getTarget),
            AutoCommands.intakeMode(superstructure, rack),
            followTrajectory("udl3", false, trajs),
            AutoCommands.shootSequence(superstructure, drive, superstructure::getTarget)),
        trajs);
  }

  private Command rightDoubleSwingAuto() {
    List<Trajectory<SwerveSample>> trajs = new ArrayList<>();
    return registerAutoTrajectories(
        Commands.sequence(
            AutoCommands.intakeMode(superstructure, rack),
            followTrajectory("d1", true, trajs),
            followTrajectory("d2", false, trajs),
            AutoCommands.shootSequence(superstructure, drive, superstructure::getTarget),
            AutoCommands.intakeMode(superstructure, rack),
            followTrajectory("d3", false, trajs),
            AutoCommands.shootSequence(superstructure, drive, superstructure::getTarget)),
        trajs);
  }

  private Command midAuto() {
    List<Trajectory<SwerveSample>> trajs = new ArrayList<>();
    return registerAutoTrajectories(
        Commands.sequence(
            followTrajectory("M1", true, trajs),
            AutoCommands.shootSequence(superstructure, drive, superstructure::getTarget)),
        trajs);
  }

  private Command midDepotAuto() {
    List<Trajectory<SwerveSample>> trajs = new ArrayList<>();
    return registerAutoTrajectories(
        Commands.sequence(
            AutoCommands.intakeMode(superstructure, rack),
            followTrajectory("MD1", true, trajs),
            followTrajectory("MD2", false, trajs),
            AutoCommands.shootSequence(superstructure, drive, superstructure::getTarget)),
        trajs);
  }
}
