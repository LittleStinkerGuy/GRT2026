package frc.robot.commands;

import frc.robot.subsystems.shooter.flywheel;
import frc.robot.subsystems.shooter.hood;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.*;

import frc.robot.commands.shooter.rampFlywheel;
import frc.robot.commands.shooter.hoodCommand;
import frc.robot.commands.hopper.indexerRun;

import frc.robot.commands.allign.AimToHubCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.FieldManagementSubsystem;

public class ShooterSequence extends SequentialCommandGroup {

    public ShooterSequence(SwerveSubsystem swerve, flywheel fly, hood hood, HopperSubsystem hopper, FieldManagementSubsystem f) {

        addCommands(
            // Aim first
            new AimToHubCommand(swerve, f),

            // Then shoot
            new ParallelCommandGroup( 
                new rampFlywheel(fly, swerve),
                new hoodCommand(hood, swerve),
                new indexerRun(hopper)
            )
        );
    }
}