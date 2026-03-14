package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.HopperConstants.HOPPER_INTAKE;
import frc.robot.Constants.TowerConstants;
import frc.robot.Constants.TowerConstants.TOWER_INTAKE;
import frc.robot.commands.allign.AimToHubCommand;
import frc.robot.commands.hopper.indexerRun;
import frc.robot.commands.shooter.SpinFlywheelCommand;
import frc.robot.commands.shooter.hoodCommand;
import frc.robot.commands.shooter.rampFlywheel;
import frc.robot.commands.shooter.towerRollers.towerRoll;
import frc.robot.commands.swerve.DriveBackwardsCommand;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.shooter.towerRollers;

public class ShootAndLeaveAuton extends SequentialCommandGroup {
    private SwerveSubsystem swerveSubsystem;
    private flywheel flySubsystem;
    private HopperSubsystem hopperSubsystem;
    private towerRollers towerSubsystem; 
    private hood hoodSubsystem;

    // Link to dimensions https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    public ShootAndLeaveAuton(SwerveSubsystem swerveSubsystem, flywheel flySubsystem, hood hoodSubsystem, HopperSubsystem hopperSubsystem, towerRollers towerSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.flySubsystem = flySubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.towerSubsystem = towerSubsystem;

        addCommands(
            new SpinFlywheelCommand(flySubsystem, 60)
                .alongWith(Commands.runOnce(() -> hoodSubsystem.setHoodAngle(0), hoodSubsystem)),

            Commands.waitSeconds(5),
            Commands.run(() -> hopperSubsystem.setHopper(HOPPER_INTAKE.BALLIN), hopperSubsystem)
            .alongWith(Commands.run(()->towerSubsystem.setTower(TOWER_INTAKE.BALLUP), towerSubsystem))
        // .withTimeout(10)
        // .andThen(new DriveBackwardsCommand(swerveSubsystem, AlignConstants.AUTON_SPEEDS)
        );

    }
}
