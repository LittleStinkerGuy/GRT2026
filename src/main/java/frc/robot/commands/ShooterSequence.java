package frc.robot.commands;

import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.FMS.FieldManagementSubsystem;
import frc.robot.subsystems.shooter.towerRollers;

import frc.robot.commands.shooter.rampFlywheel;
import frc.robot.commands.shooter.hoodCommand;
import frc.robot.commands.hopper.indexerRun;
import frc.robot.commands.shooter.towerRollers.towerRoll;
import frc.robot.commands.allign.AimToHubCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ShooterSequence extends SequentialCommandGroup {

    public ShooterSequence(SwerveSubsystem swerve, flywheel fly, hood hood, HopperSubsystem hopper, FieldManagementSubsystem fms, towerRollers b) {

        AimToHubCommand aimToHubCommand = new AimToHubCommand(swerve, fms);

        addCommands(

            new ParallelCommandGroup(
                new rampFlywheel(fly),
                new hoodCommand(hood),
                new towerRoll(b),

                new indexerRun(hopper)
                    .onlyWhile(() -> (fly.wantedVel() && hood.wantedAngl())),

                aimToHubCommand
            )
        );
    }
}