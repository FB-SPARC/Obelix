// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.bed.Bed;
import frc.robot.subsystems.bed.BedIO;
import frc.robot.subsystems.bed.BedIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.rack.Rack;
import frc.robot.subsystems.rack.RackIO;
import frc.robot.subsystems.rack.RackIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.State;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
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

  @SuppressWarnings("unused")
  private final Vision vision;

  // Controller
  private final CommandPS5Controller controller = new CommandPS5Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight-left", drive::getRotation),
                new VisionIOLimelight("limelight-right", drive::getRotation));
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

        bed = new Bed(new BedIO() {});
        feeder = new Feeder(new FeederIO() {});
        hood = new Hood(new HoodIO() {});
        intake = new Intake(new IntakeIO() {});
        rack = new Rack(new RackIO() {});
        shooter = new Shooter(new ShooterIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        break;
    }

    // Create superstructure (coordinates all non-drive subsystems + drive)
    superstructure = new Superstructure(bed, feeder, hood, intake, rack, shooter, drive);

    NamedCommands.registerCommand("Intake Mode", AutoCommands.intakeMode(superstructure, rack));
    NamedCommands.registerCommand(
        "Shoot", AutoCommands.shootSequence(superstructure, drive, superstructure::getTarget));
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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

    // ── Superstructure state bindings ──────────────────────────────────────
    // Triangle: reset rack encoder for calibration
    controller.triangle().onTrue(new InstantCommand(() -> rack.resetEncoder()));

    // R1: Intake control (press → INTAKING, release → ACTIVE if was intaking)
    controller.R1().onTrue(new InstantCommand(() -> superstructure.setState(State.INTAKING)));
    controller
        .R1()
        .onFalse(
            new InstantCommand(
                () -> {
                  if (superstructure.getState() == State.INTAKING)
                    superstructure.setState(State.ACTIVE);
                }));

    controller
        .options()
        .onTrue(new InstantCommand(() -> superstructure.setState(State.INTAKE_CLOSED)));

    // ── Shooting mode ─────────────────────────────────────────────────────
    final double DEADBAND = 0.1;

    // L1: Shooting control (press → SHOOTING, release → ACTIVE if was shooting)
    // L1 is the primary shot authority — holding it enables both state and drive auto-aim
    controller.L1().onTrue(new InstantCommand(() -> superstructure.setState(State.SHOOTING)));
    controller
        .L1()
        .onFalse(
            new InstantCommand(
                () -> {
                  if (superstructure.isShooting()) superstructure.setState(State.ACTIVE);
                }));

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
        .onTrue(new InstantCommand(() -> superstructure.setState(State.ACTIVE)));

    // Touchpad: Emergency stop (true panic button)
    // Shuts down all motors immediately regardless of state
    controller.touchpad().onTrue(new InstantCommand(() -> superstructure.setState(State.IDLE)));
  }

  public void teleopInit() {
    if (superstructure != null) superstructure.setState(State.ACTIVE);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
