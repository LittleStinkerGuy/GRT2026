// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Vision.VisionConstants;
// import frc.robot.Constants.VisionConstants;
// frc imports
import frc.robot.controllers.PS5DriveController;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
// Subsystems
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
import frc.robot.Constants.HopperConstants.HOPPER_INTAKE;
import frc.robot.commands.ManualShooterSequence;
import frc.robot.commands.ShooterSequence;
import frc.robot.commands.auton.ShootAndLeaveAuton;
import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.commands.intake.pivot.*;
import frc.robot.commands.intake.roller.*;
import frc.robot.commands.hopper.*;
import frc.robot.commands.climb.ClimbCommands.*;
import frc.robot.commands.shooter.rampDownFlywheel;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
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
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private PS5DriveController driveController;
    private CommandPS5Controller mechController;
    private final CANBus swerveCAN = new CANBus(Constants.Swerve_CAN_BUS);
    private final CANBus mechCAN = new CANBus(Constants.Mech_CAN_BUS);

    private SwerveSubsystem swerveSubsystem = Constants.SWERVE_ENABLED ? new SwerveSubsystem(swerveCAN) : null;
    private final FieldManagementSubsystem fmsSubsystem = new FieldManagementSubsystem();
    // private towerRollers tower = new towerRollers(mechCAN);

    private final RollerIntakeSubsystem intakeSubsystem = new RollerIntakeSubsystem(mechCAN);
    private final PivotIntakeSubsystem pivotIntake = new PivotIntakeSubsystem(mechCAN);
    // private final HopperSubsystem HopperSubsystem = new HopperSubsystem(mechCAN);
    private final Field2d m_field = new Field2d();
    // private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem(mechCAN);
    // private final flywheel flywheelSubsystem = new flywheel(mechCAN);
    // private final hood hoodSubsystem = new hood(mechCAN);
    private boolean shootSeq = false;

    // private final FuelDetectionSubsystem fuelDetectionSubsystem = new FuelDetectionSubsystem(VisionConstants.fuelDetectionConfig);

    // private final VisionSubsystem visionSubsystem1 = new VisionSubsystem(
    // VisionConstants.cameraConfig11);

    private UsbCamera camera1;
    private UsbCamera camera2;
    private VideoSink cameraServer;
    private boolean isCamera1Active = true;

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

        // switchable cameras
        camera1 = CameraServer.startAutomaticCapture(0);
        camera2 = CameraServer.startAutomaticCapture(1);
        cameraServer = CameraServer.getServer();

        SmartDashboard.putData("Field", m_field);
        NamedCommands.registerCommand("deployIntake", new PivotDownTimedCommand(pivotIntake));
        NamedCommands.registerCommand("runRollers", new RollerInCommand(intakeSubsystem));
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
                    // L1 = boost mode (higher accel/velocity)
                    swerveSubsystem.setBoostMode(driveController.getLeftBumper());

                    // L2 = speed limit (harder press = slower)
                    double leftTrigger = driveController.getLeftTriggerAxis();
                    double speedLimit = 1.0 - leftTrigger;
                    swerveSubsystem.setDriveSpeedLimit(speedLimit);

                    swerveSubsystem.setDrivePowers(
                        driveController.getForwardPower(),
                        driveController.getLeftPower(),
                        driveController.getRotatePower());
                },
                    swerveSubsystem));

            // Create button = switch cameras
            driveController.create().onTrue(
                Commands.runOnce(() -> {
                    if (isCamera1Active) {
                        cameraServer.setSource(camera2);
                    } else {
                        cameraServer.setSource(camera1);
                    }
                    isCamera1Active = !isCamera1Active;
                }));

            /* Pressing the button resets the field axes to the current robot axes. */
            driveController.bindDriverHeadingReset(
                () -> {
                    swerveSubsystem.resetDriverHeading();
                },
                swerveSubsystem);
        }
        if (Constants.MECH_ENABLED) {
            // bind semi auto commands
            // Cross (mech) = semi auto climb down
            var crossTrigger = mechController.cross();
            // crossTrigger.onTrue(new SemiAutoClimbDownCommand(m_ClimbSubsystem, crossTrigger::getAsBoolean));

            // Triangle (mech) = semi auto climb up
            var triangleTrigger = mechController.triangle();
            // triangleTrigger.onTrue(new SemiAutoClimbUpCommand(m_ClimbSubsystem, triangleTrigger::getAsBoolean));

            // Manual control with d-pad for winch and left stick for arm
            // m_ClimbSubsystem.setDefaultCommand(Commands.run(() -> {
            // var armDutyCycle = mechController.getLeftY();
            // double winchDutyCycle = 0;

            // if (mechController.povUp().getAsBoolean()) {
            // winchDutyCycle--;
            // }
            // if (mechController.povDown().getAsBoolean()) {
            // winchDutyCycle++;
            // }
            // m_ClimbSubsystem.setArmDutyCycle(armDutyCycle);
            // m_ClimbSubsystem.setWinchDutyCycle(winchDutyCycle);
            // }, m_ClimbSubsystem));

            // ==================== INTAKE ROLLER ====================
            // R1 (mech) = intake out, L1 (mech) = intake in (duty cycle control)
            mechController.L1().whileTrue(Commands.run(() -> intakeSubsystem.runInDutyCycle(), intakeSubsystem));
            mechController.R1().whileTrue(Commands.run(() -> intakeSubsystem.runOutDutyCycle(), intakeSubsystem));
            intakeSubsystem.setDefaultCommand(Commands.run(() -> intakeSubsystem.stop(), intakeSubsystem));

            // ==================== INTAKE PIVOT ====================
            // D-pad left = pivot down (timed), D-pad right = pivot up (timed)
            mechController.povLeft().onTrue(new PivotDownTimedCommand(pivotIntake));
            mechController.povRight().onTrue(new PivotUpTimedCommand(pivotIntake));
            pivotIntake.setDefaultCommand(Commands.run(() -> pivotIntake.stop(), pivotIntake));

            // L2 (mech) = spin spindexer (hopper) at max RPM and tower at 0.7 duty cycle
            // mechController.L2().whileTrue(Commands.run(() -> {
            // HopperSubsystem.setManualControl(1.0); // Max duty cycle for spindexer
            // tower.setManualControl(
            // 0.7); // 0.7 duty cycle for tower
            // }, HopperSubsystem, tower));
            // HopperSubsystem.setDefaultCommand(Commands.run(() -> HopperSubsystem.setManualControl(0), HopperSubsystem));

            // R2 (drive) = force intake in (pivot up + stop rollers) - hold to override
            new Trigger(driveController::getRightTrigger)
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

            // ==================== MANUAL SHOOTER SEQUENCE (SMASH AND SHOOT) ====================
            // R1 (drive) = manual shooter sequence, any other button cancels
            // mechController.square().toggleOnTrue(
            // Commands.defer(
            // () -> new ShooterSequence(
            // flywheelSubsystem,
            // hoodSubsystem,
            // tower,
            // HopperSubsystem,
            // intakeSubsystem),
            // java.util.Set.of(
            // flywheelSubsystem,
            // hoodSubsystem,
            // HopperSubsystem,
            // tower,
            // intakeSubsystem)));

            // Joystick movement cancels it
            // Trigger joystickMoved = new Trigger(() -> Math.abs(driveController.getForwardPower()) > 0.1 ||
            // Math.abs(driveController.getLeftPower()) > 0.1 ||
            // Math.abs(driveController.getRotatePower()) > 0.1);
            // joystickMoved.onTrue(Commands.runOnce(() -> manualShooterCmd.cancel()));

            // ==================== SHOOTER ====================
            // R2 = flywheel (analog speed control)
            // Left stick Y = hood manual control
            // flywheelSubsystem.setDefaultCommand(Commands.run(() -> {
            // if (DriverStation.isJoystickConnected(1)) {
            // flywheelSubsystem.flySpeed((mechController.getR2Axis() + 1) / 3);
            // } else {
            // flywheelSubsystem.flySpeed(0);
            // }
            // }, flywheelSubsystem));

            // tower.setDefaultCommand(Commands.run(() -> {
            // tower.setManualControl(0); // Stop tower by default
            // }, tower));


            // hoodSubsystem.setDefaultCommand(Commands.run(() -> {
            // if (mechController.L3().getAsBoolean()) {
            // hoodSubsystem.hoodSpeed(0.15);
            // } else if (mechController.R3().getAsBoolean()) {
            // hoodSubsystem.hoodSpeed(-0.15);
            // } else {
            // hoodSubsystem.hoodSpeed(0);
            // }
            // }, hoodSubsystem));

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

    // public Command getAutonomousCommand() {
    // // return new PathPlannerAuto("auton1");
    // return new ShootAndLeaveAuton(swerveSubsystem, flywheelSubsystem, hoodSubsystem, HopperSubsystem, tower, pivotIntake, intakeSubsystem);
    // }

    /**
     * Called when teleop starts to reset driver heading with 90 degree offset.
     */
    public void onTeleopInit() {
        if (swerveSubsystem != null) {
            swerveSubsystem.resetDriverHeadingOffset90();
        }
    }
    // return new ShootAndLeaveAuton(swerveSubsystem, flywheelSubsystem, hoodSubsystem, HopperSubsystem, tower, pivotIntake);
    // }

    // vision shit
    public void visionStuff() {
        // visionSubsystem1.setInterface(swerveSubsystem::addVisionMeasurements);

        // CommandScheduler.getInstance().schedule(
        // new GetCameraDisplacement(visionSubsystem1,
        // new Transform3d(
        // Units.inchesToMeters(0),
        // Units.inchesToMeters(-43 - 15),
        // Units.inchesToMeters(44.25),
        // new Rotation3d(0, 0, Math.PI / 2))));

    }

}
