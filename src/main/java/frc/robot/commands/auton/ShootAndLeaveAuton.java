package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.HopperConstants.HopperIntake;
import frc.robot.Constants.TowerConstants.TowerIntake;
import frc.robot.Constants.TowerConstants;
import frc.robot.commands.AutonShooterSequence;
import frc.robot.commands.ShooterSequence;
import frc.robot.commands.allign.AimToHubCommand;
import frc.robot.commands.hopper.RunIndexer;
import frc.robot.commands.intake.pivot.PivotDownTimedCommand;
import frc.robot.commands.intake.pivot.PivotOutCommand;
import frc.robot.commands.shooter.SpinFlywheelCommand;
import frc.robot.commands.shooter.towerrollers.TowerRoll;
import frc.robot.commands.shooter.HoodCommand;
import frc.robot.commands.shooter.RampFlywheel;
import frc.robot.commands.swerve.DriveBackwardsCommand;
import frc.robot.subsystems.fms.FieldManagementSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;
import frc.robot.subsystems.intake.RollerIntakeSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TowerRollersSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ShootAndLeaveAuton extends SequentialCommandGroup {
    private SwerveSubsystem swerveSubsystem;
    private FlywheelSubsystem flySubsystem;
    private HopperSubsystem hopperSubsystem;
    private TowerRollersSubsystem towerSubsystem;
    private HoodSubsystem hoodSubsystem;
    private PivotIntakeSubsystem pivotIntakeSubsystem;
    private RollerIntakeSubsystem rollerSubsystem;

    // Link to dimensions https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    public ShootAndLeaveAuton(
        SwerveSubsystem swerveSubsystem,
        FlywheelSubsystem flySubsystem,
        HoodSubsystem hoodSubsystem,
        HopperSubsystem hopperSubsystem,
        TowerRollersSubsystem towerSubsystem,
        PivotIntakeSubsystem pivotIntakeSubsystem,
        RollerIntakeSubsystem rollerSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.flySubsystem = flySubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.towerSubsystem = towerSubsystem;
        this.pivotIntakeSubsystem = pivotIntakeSubsystem;
        this.rollerSubsystem = rollerSubsystem;

        addCommands(
            new AutonShooterSequence(
                flySubsystem,
                hoodSubsystem,
                towerSubsystem,
                hopperSubsystem,
                pivotIntakeSubsystem).withTimeout(3.0)
        // new SpinFlywheelCommand(flySubsystem, .36)
        // .alongWith(
        // Commands.runOnce(() -> hoodSubsystem.setHoodAngle(0), hoodSubsystem),
        // Commands.waitSeconds(5)
        // .andThen(Commands.run(() -> hopperSubsystem.setHopper(HopperIntake.BALLIN), hopperSubsystem)
        // .alongWith(Commands.run(() -> towerSubsystem.setTower(TowerIntake.BALLUP), towerSubsystem)))
        // // .withTimeout(10)
        // .andThen(new DriveBackwardsCommand(swerveSubsystem, AlignConstants.AUTON_SPEEDS)
        );

    }
}
