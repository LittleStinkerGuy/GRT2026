// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Vision.VisionConstants;
// import frc.robot.Constants.VisionConstants;
// frc imports
import frc.robot.controllers.PS5DriveController;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.shooterLearner;
// Subsystems
import frc.robot.subsystems.swerve.AimSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
// import frc.robot.subsystems.Vision.VisionSubsystem;
// import frc.robot.subsystems.Vision.CameraConfig;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.shooter.towerRollers;

import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
import frc.robot.subsystems.Vision.FuelDetectionSubsystem;
// import frc.robot.Constants.IntakeConstants;

// Commands
import frc.robot.commands.vision.GetCameraDisplacement;
import frc.robot.Constants.TowerConstants.TOWER_INTAKE;
import frc.robot.Constants.CycleShooterConstants;
import frc.robot.Constants.HopperConstants.HOPPER_INTAKE;
import frc.robot.Constants.ShooterConstants.Flywheel;
import frc.robot.commands.AutonShooterSequence;
import frc.robot.commands.CycleShot;
import frc.robot.commands.SmashShot;
import frc.robot.commands.TowerShot;
import frc.robot.commands.allign.AimToHubCommand;
import frc.robot.commands.shooter.CalibrationCommands;
import frc.robot.commands.ShooterSequence;
import frc.robot.commands.cycleBallsCommand;
import frc.robot.commands.auton.ANeutralIntakeAuton;
import frc.robot.commands.auton.CNeutralIntakeAuton;
import frc.robot.commands.auton.CenterAuton;
import frc.robot.commands.auton.ShootAndLeaveAuton;
import frc.robot.commands.auton.ToDepotAndShoot;
import frc.robot.commands.auton.Turn90AutonPP;
import frc.robot.commands.auton.NeutralDefenseAuton;
import frc.robot.commands.auton.TIMECNeutralIntakeTOWERAuton;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.commands.intake.pivot.*;
import frc.robot.commands.intake.roller.*;
import frc.robot.commands.intake.PivotAndRollerIntakeCommand;
import frc.robot.commands.hopper.*;
import frc.robot.commands.shooter.rampDownFlywheel;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

// WPILib imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private double cycleFlywheelVelo = CycleShooterConstants.FLYWHEEL_RPS;
    private DoubleSupplier cycleFlywheelOffsetGetter = () -> (cycleFlywheelVelo - CycleShooterConstants.FLYWHEEL_RPS);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private PS5DriveController driveController;
    private CommandPS5Controller mechController;
    private final CANBus swerveCAN = new CANBus(Constants.Swerve_CAN_BUS);
    private final CANBus mechCAN = new CANBus(Constants.Mech_CAN_BUS);

    private SwerveSubsystem swerveSubsystem = Constants.SWERVE_ENABLED ? new SwerveSubsystem(swerveCAN) : null;
    private final FieldManagementSubsystem fmsSubsystem = new FieldManagementSubsystem(cycleFlywheelOffsetGetter);
    private towerRollers tower = new towerRollers(mechCAN);

    private final RollerIntakeSubsystem intakeSubsystem = new RollerIntakeSubsystem(mechCAN);
    private final PivotIntakeSubsystem pivotIntake = new PivotIntakeSubsystem(mechCAN);
    private final HopperSubsystem HopperSubsystem = new HopperSubsystem(mechCAN);
    private final Field2d m_field = new Field2d();
    private final flywheel flywheelSubsystem = new flywheel(mechCAN);
    private final hood hoodSubsystem = new hood(mechCAN);
    private final shooterLearner learner = new shooterLearner();
    private final AimSubsystem aimSubsystem =
        (Constants.SWERVE_ENABLED && swerveSubsystem != null)
            ? new AimSubsystem(swerveSubsystem, fmsSubsystem)
            : null;
    private final AimToHubCommand aimHelper =
        (Constants.SWERVE_ENABLED && swerveSubsystem != null)
            ? new AimToHubCommand(swerveSubsystem, fmsSubsystem)
            : null;
    private boolean shootSeq = false;

    // private final FuelDetectionSubsystem fuelDetectionSubsystem = new FuelDetectionSubsystem(VisionConstants.fuelDetectionConfig);

    private final VisionSubsystem visionSubsystem1 = new VisionSubsystem(
        VisionConstants.cameraConfig1);
    private final VisionSubsystem visionSubsystem2 = new VisionSubsystem(
        VisionConstants.cameraConfig2);
    private final VisionSubsystem visionSubsystem3 = new VisionSubsystem(
        VisionConstants.cameraConfig3);
    private UsbCamera driverCam;


    private double desiredHoodSpeed = 0;
    // private final VisionSubsystem visionSubsystem1 = new VisionSubsystem(
    // VisionConstants.cameraConfig11);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        visionStuff();
        constructController();
        configureBindings();
        configureAutoChooser();

        // Driver cam — publishes to NT at /CameraPublisher/Driver Cam/streams
        driverCam = CameraServer.startAutomaticCapture("Driver Cam", 0);
        driverCam.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);

        // Throttle what actually streams to the dashboard (the source-side FPS
        // is just a hint — Arducams often ignore it). These caps are authoritative.
        MjpegServer driverCamServer = (MjpegServer) CameraServer.getServer("serve_Driver Cam");
        driverCamServer.setResolution(320, 240);
        driverCamServer.setFPS(20);
        driverCamServer.setCompression(50); // 0 = worst, 100 = best; lower = less bandwidth

        SmartDashboard.putData("Field", m_field);
        NamedCommands.registerCommand("deployIntake", new PivotDownTimedCommand(pivotIntake));
        NamedCommands.registerCommand("runRollers", new RollerInCommand(intakeSubsystem));
        NamedCommands.registerCommand("pivotAndRollerIntake", new PivotAndRollerIntakeCommand(pivotIntake, intakeSubsystem));
        NamedCommands.registerCommand("pivotdownandrunrollers", new PivotAndRollerIntakeCommand(pivotIntake, intakeSubsystem));
        NamedCommands.registerCommand("shootSequence", new AutonShooterSequence(flywheelSubsystem, hoodSubsystem, tower, HopperSubsystem, pivotIntake));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named f`actories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */

    private void configureBindings() {
        /*
         * Driving -- One joystick controls translation, the other rotation. If the
         * robot-relative button is held down,
         * the robot is controlled along its own axes, otherwise controls apply to the
         * field axes by default. If the
         * swerve aim button is held down, the robot will rotate automatically to always
         * face a target, and only
         * translation will be manually controllable.
         */
        if (Constants.SWERVE_ENABLED && swerveSubsystem != null) {
            swerveSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    // R2 (>= 90% pressed) = boost mode
                    swerveSubsystem.setBoostMode(driveController.getRightTriggerAxis() > 0.9);

                    // L1 (hold) = robot-relative drive; release returns to field-relative
                    swerveSubsystem.setRobotRelative(driveController.getLeftBumper());

                    // R1 = slow mode (30% speed), L2 = variable speed limit
                    double speedLimit;
                    if (driveController.getRightBumper()) {
                        speedLimit = Constants.SwerveConstants.SLOW_MODE_SPEED_LIMIT;
                    } else {
                        double leftTrigger = driveController.getLeftTriggerAxis();
                        speedLimit = 1.0 - leftTrigger;
                    }
                    swerveSubsystem.setDriveSpeedLimit(speedLimit);

                    swerveSubsystem.setDrivePowers(
                        driveController.getForwardPower(),
                        driveController.getLeftPower(),
                        driveController.getRotatePower());
                },
                    swerveSubsystem));

            // Create button = switch cameras
            // driveController.create().onTrue(
            // Commands.runOnce(() -> {
            // if (isCamera1Active) {
            // cameraServer.setSource(camera2);
            // } else {
            // cameraServer.setSource(camera1);
            // }
            // isCamera1Active = !isCamera1Active;
            // }));

            /* Pressing the button resets the field axes to the current robot axes. */
            driveController.bindDriverHeadingReset(
                () -> {
                    swerveSubsystem.resetDriverHeading();
                },
                swerveSubsystem);

            // Triangle (drive) = auto-rotate to face the hub. Held-while-true; rotating
            // the right stick past the deadband cancels it so the driver can override.
            // Wrapped in Commands.defer so the target angle is recomputed at every press.
            if (aimHelper != null) {
                driveController.getController().triangle().whileTrue(
                    Commands.defer(
                        () -> aimHelper.createAimCommand(() -> false),
                        java.util.Set.of(swerveSubsystem)));
            }

            driveController.getController().L2().whileTrue(
                Commands.run(() -> {
                    hoodSubsystem.setHoodAngle(0);
                }, hoodSubsystem));
        }
        if (Constants.MECH_ENABLED) {
            // ==================== INTAKE ROLLER ====================
            // R1 (mech) = intake out, L1 (mech) = intake in (duty cycle control)
            mechController.L1().whileTrue(Commands.run(() -> intakeSubsystem.runIn(), intakeSubsystem));
            mechController.R1().whileTrue(Commands.run(() -> intakeSubsystem.runOut(), intakeSubsystem));
            intakeSubsystem.setDefaultCommand(Commands.run(() -> intakeSubsystem.stop(), intakeSubsystem));

            // ==================== INTAKE PIVOT ====================
            // D-pad left = pivot down (timed), D-pad right = pivot up (timed)
            mechController.povLeft().whileTrue(new PivotOutCommand(pivotIntake));
            mechController.povRight().whileTrue(new PivotInCommand(pivotIntake));

            // L2 (mech) = spin spindexer (hopper) at max RPM and tower at full duty cycle
            mechController.L2().whileTrue(Commands.run(() -> {
                HopperSubsystem.setManualControl(-1.0); // Max duty cycle for spindexer
                tower.setManualControl(1.0); // Full duty cycle for tower
            }, HopperSubsystem, tower));
            HopperSubsystem.setDefaultCommand(Commands.run(() -> HopperSubsystem.setManualControl(0), HopperSubsystem));

            // Square (drive) = emergency force intake in (pivot up + stop rollers) - hold to override
            driveController.square()
                .whileTrue(Commands.run(() -> {
                    pivotIntake.setPosition(Constants.IntakeConstants.PIVOT_IN_POS);
                    intakeSubsystem.stop();
                }, pivotIntake, intakeSubsystem));

            // ==================== INTAKE PIVOT ====================
            // Right stick Y controls pivot manually
            /*
             * pivotIntake.setDefaultCommand(Commands.run(() -> {
             * double pivotInput = -mechController.getRightY();
             * if (Math.abs(pivotInput) > 0.1) {
             * pivotIntake.setManualSpeed(pivotInput * 0.3);
             * } else {
             * pivotIntake.stop();
             * }
             * }, pivotIntake));
             */

            // ==================== SHOOTING PRESETS ====================
            // Square (mech) = smash-and-shoot preset (close-range)
            mechController.square().whileTrue(
                Commands.defer(
                    () -> new SmashShot(
                        flywheelSubsystem,
                        hoodSubsystem,
                        tower,
                        HopperSubsystem,
                        pivotIntake,
                        learner),
                    java.util.Set.of(
                        flywheelSubsystem,
                        hoodSubsystem,
                        HopperSubsystem,
                        tower,
                        pivotIntake)));

            // ==================== INTERPOLATION TABLE CALIBRATION ====================
            // D-pad up/down: bump flywheel RPM offset (5 RPS per press)
            // Triangle/Circle: bump hood angle offset (0.005 rotations per press)
            // Options: log current (distance, rpm, angle) to riolog
            // Share: reset both offsets to zero
            // mechController.povUp().onTrue(CalibrationCommands.rpmUp(learner));
            // mechController.povDown().onTrue(CalibrationCommands.rpmDown(learner));
            // mechController.triangle().onTrue(CalibrationCommands.hoodUp(learner));
            // mechController.circle().onTrue(CalibrationCommands.hoodDown(learner));
            // mechController.create().onTrue(CalibrationCommands.resetOffsets(learner));
            // if (aimSubsystem != null) {
            // mechController.options().onTrue(CalibrationCommands.logPoint(learner, aimSubsystem));
            // }

            // Joystick movement cancels it
            // Trigger joystickMoved = new Trigger(() -> Math.abs(driveController.getForwardPower()) > 0.1 ||
            // Math.abs(driveController.getLeftPower()) > 0.1 ||
            // Math.abs(driveController.getRotatePower()) > 0.1);
            // joystickMoved.onTrue(Commands.runOnce(() -> manualShooterCmd.cancel()));

            // ==================== SHOOTER ====================
            // R2 = flywheel (analog speed control)
            // Left stick Y = hood manual control
            flywheelSubsystem.setDefaultCommand(Commands.run(() -> {
                if (DriverStation.isJoystickConnected(1)) {
                    flywheelSubsystem.flySpeed((mechController.getR2Axis() + 1) / 3);
                } else {
                    flywheelSubsystem.flySpeed(0);
                }
            }, flywheelSubsystem));

            tower.setDefaultCommand(Commands.run(() -> {
                tower.setManualControl(0); // Stop tower by default
            }, tower));

            hoodSubsystem.setDefaultCommand(Commands.run(() -> {
                if (mechController.L3().getAsBoolean()) {
                    desiredHoodSpeed = 0.15;
                    hoodSubsystem.hoodSpeed(0.15);
                } else if (mechController.R3().getAsBoolean()) {
                    desiredHoodSpeed = -0.15;
                    hoodSubsystem.hoodSpeed(-0.15);
                } else {
                    if (desiredHoodSpeed != 0) {
                        desiredHoodSpeed = 0;
                        hoodSubsystem.hoodSpeed(0);

                    }
                }
            }, hoodSubsystem));

            mechController.povUp().onTrue(Commands.runOnce(() -> {
                if (cycleFlywheelVelo < Flywheel.FLYWHEEL_MAX_SPEED) {
                    cycleFlywheelVelo += 5;
                }
            }));

            mechController.povDown().onTrue(Commands.runOnce(() -> {
                if (cycleFlywheelVelo > 0) {
                    cycleFlywheelVelo -= 5;
                }
            }));

            // Cross = passing shot — flywheel pinned to max (120 RPS)
            mechController.cross().whileTrue(new CycleShot(
                flywheelSubsystem,
                hoodSubsystem,
                tower,
                HopperSubsystem,
                () -> Flywheel.FLYWHEEL_MAX_SPEED));

            // Touchpad = tower shoot preset
            mechController.triangle().whileTrue(new TowerShot(
                flywheelSubsystem,
                hoodSubsystem,
                tower,
                HopperSubsystem,
                pivotIntake,
                learner));

            // Swerve-dependent drive controller commands
            if (Constants.SWERVE_ENABLED && swerveSubsystem != null) {
                // Options button = reset pose to starting position (in front of red hub)
                driveController.options()
                    .onTrue(Commands.runOnce(() -> swerveSubsystem.resetToStartingPosition(), swerveSubsystem));
            }
        }

    }

    /**
     * Constructs the drive controller based on the name of the controller at port
     * 0
     */
    private void constructController() {
        driveController = new PS5DriveController();
        driveController.setDeadZone(0.035);
        mechController = new CommandPS5Controller(1);
    }

    /**
     * Config the autonomous command chooser
     */
    private void configureAutoChooser() {
        // Add auton here
        autoChooser.setDefaultOption("Do Nothing", null);

        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("isolatedINTAKE");
        // return new PathPlannerAuto("auton2");
        // return new NeutralDefenseAuton(flywheelSubsystem, hoodSubsystem, tower, HopperSubsystem, pivotIntake, intakeSubsystem);
        // return new ANeutralIntakeAuton(flywheelSubsystem, hoodSubsystem, tower, HopperSubsystem, pivotIntake, intakeSubsystem);
        // return new PathPlannerAuto("auton1");
        // return new ShootAndLeaveAuton(swerveSubsystem, flywheelSubsystem, hoodSubsystem, HopperSubsystem, tower, pivotIntake, intakeSubsystem);

        // Run ManualShooterSequence for 10 seconds
        // return new AutonShooterSequence(
        // flywheelSubsystem,
        // hoodSubsystem,
        // tower,
        // HopperSubsystem,
        // pivotIntake).withTimeout(10);

    }

    /**
     * Called when teleop starts to reset driver heading with 90 degree offset.
     */
    public void onTeleopInit() {
        if (swerveSubsystem != null) {
            swerveSubsystem.resetDriverHeadingOffset90();
        }
    }

    /**
     * Called when autonomous starts to zero the pivot encoder.
     */
    public void onAutonInit() {
        // pivotIntake.zeroEncoder();
    }
    // return new ShootAndLeaveAuton(swerveSubsystem, flywheelSubsystem, hoodSubsystem, HopperSubsystem, tower, pivotIntake);
    // }

    // vision shit
    public void visionStuff() {
        visionSubsystem1.setInterface(swerveSubsystem::addVisionMeasurements);
        visionSubsystem2.setInterface(swerveSubsystem::addVisionMeasurements);
        visionSubsystem3.setInterface(swerveSubsystem::addVisionMeasurements);

        // CommandScheduler.getInstance().schedule(
        // new GetCameraDisplacement(visionSubsystem1,
        // new Transform3d(
        // Units.inchesToMeters(0),
        // Units.inchesToMeters(-43 - 15),
        // Units.inchesToMeters(44.25),
        // new Rotation3d(0, 0, Math.PI / 2))));

    }

}
