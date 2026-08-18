// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANType;
import frc.robot.Constants.CycleShooterConstants;
import frc.robot.commands.CycleShot;
import frc.robot.commands.SmashShot;
import frc.robot.commands.TowerShot;
import frc.robot.commands.auton.ShootAndLeaveAuton;
import frc.robot.controllers.PS5DriveController;
import frc.robot.subsystems.fms.FieldManagementSubsystem;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.hopper.HopperIOTalonFXSim;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotIOTalonFX;
import frc.robot.subsystems.intake.pivot.PivotIOTalonFXSim;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerIOTalonFX;
import frc.robot.subsystems.intake.roller.RollerIOTalonFXSim;
import frc.robot.subsystems.intake.roller.RollerSubsystem;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFXSim;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFXSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.tower.TowerIO;
import frc.robot.subsystems.shooter.tower.TowerIOTalonFX;
import frc.robot.subsystems.shooter.tower.TowerIOTalonFXSim;
import frc.robot.subsystems.shooter.tower.TowerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.LoggedCanivore;
import frc.robot.util.TracerSentinel;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.MutAngularVelocity;
import java.util.function.DoubleSupplier;

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
    // Must be the first SubsystemBase constructed
    // Captures pre-subsystem scheduler overhead
    @SuppressWarnings("unused")
    private final TracerSentinel tracerSentinel = new TracerSentinel();

    private double cycleFlywheelVelo = CycleShooterConstants.FLYWHEEL_VELO.in(RotationsPerSecond);
    private DoubleSupplier cycleFlywheelOffsetGetter = () -> (cycleFlywheelVelo - CycleShooterConstants.FLYWHEEL_VELO.in(RotationsPerSecond));

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private PS5DriveController driveController;
    private CommandPS5Controller driveControllerReal;
    private final LoggedCanivore swerveCan = new LoggedCanivore(CANType.SWERVE);
    private final LoggedCanivore mechCan = new LoggedCanivore(CANType.MECH);

    private SwerveSubsystem swerveSubsystem = Constants.SWERVE_ENABLED ? new SwerveSubsystem(swerveCan) : null;
    private final FieldManagementSubsystem fmsSubsystem = new FieldManagementSubsystem(cycleFlywheelOffsetGetter);

    private final PivotSubsystem pivot;
    private final RollerSubsystem roller;
    private final HopperSubsystem hopper;
    private final TowerSubsystem tower;
    private final FlywheelSubsystem flywheel;
    private final HoodSubsystem hood;

    private double desiredHoodSpeed = 0;
    private final MutAngularVelocity flywheelManualVeloCommand = RotationsPerSecond.mutable(0.0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                pivot = new PivotSubsystem(new PivotIOTalonFX(mechCan));
                roller = new RollerSubsystem(new RollerIOTalonFX(mechCan));
                hopper = new HopperSubsystem(new HopperIOTalonFX(mechCan));
                tower = new TowerSubsystem(new TowerIOTalonFX(mechCan));
                flywheel = new FlywheelSubsystem(new FlywheelIOTalonFX(mechCan));
                hood = new HoodSubsystem(new HoodIOTalonFX(mechCan));
                break;
            case SIM:
                pivot = new PivotSubsystem(new PivotIOTalonFXSim(mechCan));
                roller = new RollerSubsystem(new RollerIOTalonFXSim(mechCan));
                hopper = new HopperSubsystem(new HopperIOTalonFXSim(mechCan));
                tower = new TowerSubsystem(new TowerIOTalonFXSim(mechCan));
                flywheel = new FlywheelSubsystem(new FlywheelIOTalonFXSim(mechCan));
                hood = new HoodSubsystem(new HoodIOTalonFXSim(mechCan));
                break;
            case REPLAY:
            default:
                pivot = new PivotSubsystem(new PivotIO() {});
                roller = new RollerSubsystem(new RollerIO() {});
                hopper = new HopperSubsystem(new HopperIO() {});
                tower = new TowerSubsystem(new TowerIO() {});
                flywheel = new FlywheelSubsystem(new FlywheelIO() {});
                hood = new HoodSubsystem(new HoodIO() {});
                break;
        }
        constructController();
        configureBindings();
        configureAutoChooser();

        driveControllerReal.setRumble(RumbleType.kBothRumble, 1);
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
        if (Constants.SWERVE_ENABLED && swerveSubsystem != null) {
            swerveSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    swerveSubsystem.setDriveSpeedLimit(1.0);
                    swerveSubsystem.setDrivePowers(
                        driveController.getForwardPower(),
                        driveController.getLeftPower(),
                        driveController.getRotatePower());
                }, swerveSubsystem));

            /* Pressing the button resets the field axes to the current robot axes. */
            driveController.bindDriverHeadingReset(
                () -> {
                    swerveSubsystem.resetDriverHeading();
                }, swerveSubsystem);
        }
        if (Constants.MECH_ENABLED) {
            driveControllerReal.R1().whileTrue(new CycleShot(flywheel, hood, tower, hopper, cycleFlywheelOffsetGetter));
            driveControllerReal.R2().whileTrue(roller.runRollerOut());

            driveControllerReal.L1().toggleOnTrue(pivot.togglePivot());
            driveControllerReal.L2().whileTrue(roller.runRollerIn());

            driveControllerReal.triangle().toggleOnTrue(new SmashShot(flywheel, hood, tower, hopper, pivot));
            driveControllerReal.circle().toggleOnTrue(new TowerShot(flywheel, hood, tower, hopper, pivot));


            driveControllerReal.povDown().whileTrue(flywheel.setFlywheelManualSpeed(() -> -1.0));
            driveControllerReal.povUp().whileTrue(flywheel.setFlywheelManualSpeed(() -> 1.0));

            driveControllerReal.povLeft().whileTrue(hopper.runHopperOut());
            driveControllerReal.povRight().whileTrue(hopper.runHopperIn());

            roller.setDefaultCommand(roller.stopRoller());
            hopper.setDefaultCommand(hopper.stopHopper());
            tower.setDefaultCommand(tower.stopTower());

            driveControllerReal.square().toggleOnTrue(Commands.parallel(pivot.jigglePivot(), hood.jiggleHood()));
        }
    }

    /**
     * Constructs the drive controller based on the name of the controller at port 0
     */
    private void constructController() {
        driveController = new PS5DriveController();
        driveControllerReal = driveController.getController();
        driveController.setDeadZone(0.035);
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
        return new ShootAndLeaveAuton(swerveSubsystem, flywheel, hood, hopper, tower, pivot, roller);

        // return new ToDepotAndShoot(flywheel, hoodSubsystem, tower, hopper, pivotIntake, intakeSubsystem, learner);
        // return new PathPlannerAuto("90degturn");

        // rehturn new PathPlannerAuto("swerve90");

        // return new CNeutralIntakeTOWERAuton(flywheel, hoodSubsystem, tower, hopper, pivotIntake, intakeSubsystem, learner);
        // return new PathPlannerAuto("auton2");
        // return new NeutralDefenseAuton(flywheel, hoodSubsystem, tower, hopper, pivotIntake, intakeSubsystem);
        // return new ANeutralIntakeAuton(flywheel, hoodSubsystem, tower, hopper, pivotIntake, intakeSubsystem);
        // return new PathPlannerAuto("auton1");

        // Run ManualShooterSequence for 10 seconds
        // return new AutonShooterSequence(
        // flywheel,
        // hoodSubsystem,
        // tower,
        // hopper,
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
    // return new ShootAndLeaveAuton(swerveSubsystem, flywheel, hoodSubsystem, hopper, tower, pivotIntake);
    // }

}
