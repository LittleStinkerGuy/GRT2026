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
import frc.robot.commands.allign.AimWhileDrivingCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.function.DoubleSupplier;

public class ShooterSequence extends SequentialCommandGroup {

    public ShooterSequence(
        SwerveSubsystem swerve,
        flywheel fly,
        hood hood,
        HopperSubsystem hopper,
        FieldManagementSubsystem fms,
        towerRollers b,
        DoubleSupplier xSpeed,
        DoubleSupplier ySpeed
    ) {

        boolean redTeam = fms.isRedAlliance();

        AimWhileDrivingCommand aimWhileDriving =
            new AimWhileDrivingCommand(
                swerve,
                fms,
                redTeam,
                xSpeed,
                ySpeed
            );

        addCommands(

            new ParallelCommandGroup(
                aimWhileDriving,
                new rampFlywheel(fly, redTeam),
                new hoodCommand(hood, redTeam),
                new towerRoll(b),

                new indexerRun(hopper)
                    .onlyWhile(() -> (fly.wantedVel() && hood.wantedAngl() && b.correctRoll()))
            )
        );
    }
}
